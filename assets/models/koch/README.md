# Koch 9x9 预训练模型

来源：references/Koch_Gomoku_robot/Gomoku_AI/Vanilla_MCTS/save/model_9/Vanilla_MCTS.model，原样复制，MIT，Copyright (c) 2024 hri5-boardgame-robot。许可证见 src/so101_gomoku/ai/vendor/koch/LICENSE。

仅9×9、5通道网络可加载；项目已删除旧机械臂限制通道，不保证原训练分布或棋力。使用torch.load(weights_only=True,map_location='cpu')加载。测试仅确认推理、搜索和合法输出，不是棋力评估。
