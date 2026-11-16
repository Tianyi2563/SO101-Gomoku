"""External chat/completions -> actual tool_calls -> allowlisted local dispatch."""
import argparse
from getpass import getpass
from datetime import datetime
import json
import os
from pathlib import Path
import sys
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import Request, urlopen

from .tools import RobotTools, TOOLS
from so101_demo.credentials import get_glm_api_key

PROMPT = Path(__file__).with_name("prompt.txt").read_text(encoding="utf-8")
DEFAULT_TASK = "先查询状态，再用IK让末端沿基座+y移动2毫米，查询确认，最后停止并总结。必须通过工具完成，说明是Mock模拟。"


def external_chat(messages, key, model, url):
    if urlparse(url).scheme != "https":
        raise ValueError("External API must use HTTPS")
    request = Request(url, data=json.dumps(dict(model=model, messages=messages,
        tools=TOOLS, tool_choice="auto", stream=False)).encode("utf-8"),
        headers={"Authorization": "Bearer " + key, "Content-Type": "application/json"}, method="POST")
    try:
        with urlopen(request, timeout=60) as response:
            data = json.load(response)
        raw = data["choices"][0]["message"]
        result = {"role": "assistant", "content": raw.get("content")}
        if raw.get("tool_calls"):
            result["tool_calls"] = raw["tool_calls"]
        return result
    except HTTPError as exc:
        try:
            error = json.loads(exc.read()).get('error', {})
            details = str(error.get('code', '')) + ': ' + str(error.get('message', ''))
            details = details.replace(key, '[REDACTED]')[:250]
        except Exception:
            details = 'unreadable provider error'
        raise RuntimeError(f"External LLM HTTP {exc.code}; {details}") from None
    except (URLError, TimeoutError, KeyError, IndexError, json.JSONDecodeError):
        raise RuntimeError("External LLM connection or response failed") from None


def run(task, tools, chat, trace, max_rounds=10):
    messages = [{"role": "system", "content": PROMPT}, {"role": "user", "content": task}]
    calls = 0
    def event(kind, **fields):
        trace.write(json.dumps(dict(event=kind, **fields), ensure_ascii=False) + "\n")
        trace.flush()
    try:
        for turn in range(max_rounds):
            event("llm_request", turn=turn, message_count=len(messages), tools=[t['function']['name'] for t in TOOLS])
            message = chat(messages)
            event("llm_response", turn=turn, message=message)
            messages.append(message)
            requested = message.get("tool_calls", [])
            if not requested:
                if calls == 0:
                    raise RuntimeError("LLM returned text without calling any tool; acceptance failed")
                event("completed", tool_calls=calls, final=message.get("content"))
                return message.get("content") or "工具调用已完成。"
            if len(requested) > 8 or calls + len(requested) > 30:
                raise RuntimeError("Tool call budget exceeded")
            ids = [c.get('id') for c in requested]
            if any(not isinstance(i, str) or not i for i in ids) or len(set(ids)) != len(ids):
                raise RuntimeError("Invalid tool call identifiers")
            for call in requested:
                fn = call.get("function", {})
                try:
                    arguments = json.loads(fn.get("arguments", ""))
                except (ValueError, TypeError):
                    tools.stop_motion()
                    result = dict(ok=False, error="Invalid JSON arguments", stopped=True)
                else:
                    result = tools.dispatch(fn.get("name"), arguments)
                calls += 1
                event("tool_result", tool_call_id=call['id'], name=fn.get('name'), result=result)
                messages.append(dict(role="tool", tool_call_id=call['id'], content=json.dumps(result, ensure_ascii=False)))
        raise RuntimeError("Round limit reached; acceptance incomplete")
    except Exception as exc:
        event('failed', error=type(exc).__name__)
        raise
    finally:
        event("session_cleanup", result=tools.stop_motion())
        tools.close()


def main():
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description="External LLM tool-calling demonstration; hardware disabled")
    parser.add_argument("--task", default=DEFAULT_TASK)
    parser.add_argument("--model", default=os.getenv("LLM_MODEL", "glm-4-flash"))
    parser.add_argument("--api-url", default=os.getenv("LLM_API_URL", "https://open.bigmodel.cn/api/paas/v4/chat/completions"))
    parser.add_argument("--allow-camera", action="store_true")
    parser.add_argument("--ask-key", action="store_true", help="Read API key without echo; never save it")
    args = parser.parse_args()
    key = getpass("API key (hidden): ") if args.ask_key else (os.getenv("LLM_API_KEY") or get_glm_api_key())
    if not key:
        parser.error("Set GLM_API_KEY or LLM_API_KEY locally; external API acceptance NOT run")
    directory = Path(__file__).with_name("runs") / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    directory.mkdir(parents=True)
    tools = RobotTools(allow_camera=args.allow_camera)
    with (directory / "trace.jsonl").open("x", encoding="utf-8") as trace:
        trace.write(json.dumps(dict(event="run_config", external_api=True, model=args.model,
            api_url=args.api_url, mode="mock", task=args.task), ensure_ascii=False) + "\n")
        answer = run(args.task, tools, lambda messages: external_chat(messages, key, args.model, args.api_url), trace)
    print(answer)
    print("调用证据：", directory / "trace.jsonl")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
