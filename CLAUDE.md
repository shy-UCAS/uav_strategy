## 关联工程：NLTaskOrchestration（项目A）

- 位置：当前项目的同级目录 `../NLTaskOrchestration`
  （绝对路径 F:\CASIA\Drone Swarm Situational Awareness Algorithm\NLTaskOrchestration）
- 当我提到"项目A"、"NL项目"、"编排项目"时，均指该目录
- 该项目把自然语言作战指令编排成任务 DAG，再经其转换器
  `tools/export_uav_strategy_inputs.py` 导出为本项目可直接消费的三件套 JSON
  （digraph_attrs.json / key_paths.json / facilities.json）
- 导出数据已落在本项目 `examples/uavs_strategy/data/nl_export/<case_id>/`，
  数据契约与接入步骤以本项目内已拷贝的文档为准（优先读任务书）：
  - `examples/uavs_strategy/uav_strategy_bside_task_brief.md`
  - `examples/uavs_strategy/uav_strategy_integration_guide.md`
  - `examples/uavs_strategy/uav_strategy_enrichment_guide.md`
- 一般不需要读项目A的代码；仅当对数据契约有疑问且上述文档解释不了时，
  再去项目A查证
- 查找项目A内容时，先用 Glob/Grep 确认文件实际位置，不要凭猜测构造路径
- 该目录仅作只读参考，禁止修改其中任何文件；所有改动只发生在当前项目
  （uav_strategy）内
