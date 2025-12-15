can_task_start(true).
!start.

+!start
    <-
    ?cur_nodes(Start, End); 
    .print("Mission Start: ", Start, " -> ", End);
    !task_digraph(Start, End).

+!task_digraph(CurrentStart, CurrentEnd) 
    <- 
    while(true) {
        ?cur_nodes(Start, End); 
        !act_digraph_path_planning(Start, End);
    }.

+!act_digraph_path_planning(CurrentStart, CurrentEnd): can_task_start(true)
    <-
    -can_task_start(true);
    +can_task_start(false);
    .print("Planning segment: ", CurrentStart, " -> ", CurrentEnd);
    .act_digraph_path_planning(CurrentStart, CurrentEnd).

+!act_digraph_path_planning(CurrentStart, CurrentEnd): can_task_start(false)
    <-
    .wait(100);
    .print("Waiting to start path planning for segment: ", CurrentStart, " -> ", CurrentEnd).

+!test_set_intention
    <-
    .print("Test set intention").