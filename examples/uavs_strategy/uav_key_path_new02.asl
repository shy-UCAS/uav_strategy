!start.

+!start 
    <- .print("Agent active. Waiting for cur_nodes belief...").

// 核心逻辑：当 cur_nodes 信念被 Python 更新时，自动触发此计划
+cur_nodes(Start, End)
    <- 
    .print("New segment detected: ", Start, " -> ", End);
    !act_digraph_path_planning(Start, End).

+!act_digraph_path_planning(Start, End)
    <-
    .print("Requesting planning for: ", Start, " -> ", End);
    // 调用 Python action，它会立即返回并在后台启动延时任务
    .act_digraph_path_planning(Start, End).
