# SPEC
Target: 实现可运行的五子棋 AI 与离线完整回合调度，复用 Koch GitHub 神经网络/MCTS，另提供 Alpha-Beta。
Inputs: GameState、BoardObservation、上游本地 MIT 源码及9×9模型；注入的观测/执行接口。
Outputs: ai/ 与 orchestration/ 业务代码、离线多回合 demo、tests 和中文接入文档。
Base commit: e742a04ba58495ab8fe230e94a3d23fe656839b5。
Success: python -m unittest discover -s tests -v；python scripts/demos/play_gomoku.py --engine alphabeta --rounds 5；--engine koch-mcts --rounds 2。
Cuts: 不实现相机分类或硬件标定，不连接硬件，不宣称AI棋力或真实机器人成功率。
