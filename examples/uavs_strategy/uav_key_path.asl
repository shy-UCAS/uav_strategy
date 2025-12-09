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
        !act_digraph_path_planning(CurrentStart, CurrentEnd);
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




// +!task_digraph(CurrentStart, CurrentEnd) : can_task_start(true)
//     <-
//     -can_task_start(true);
//     +can_task_start(false);
//     ?next_node(LastNode, NextTarget);
//     .print("next node:",LastNode,"->",NextTarget);
//     .print("Planning segment: ", CurrentStart, " -> ", CurrentEnd);
//     .act_digraph_path_planning(CurrentStart, CurrentEnd);
//     !find_and_recurse(CurrentEnd).

// +!task_digraph(CurrentStart, CurrentEnd) : can_task_start(false)
//     <-
//     .print("Waiting to start path planning for segment: ", CurrentStart, " -> ", CurrentEnd);
//     .wait(100);
//     !task_digraph(CurrentStart, CurrentEnd).

// +!find_and_recurse(LastNode) : next_node(LastNode, NextTarget)
//     <-
//     .print("Found next segment starting from ", LastNode, " to ", NextTarget);
//     !task_digraph(LastNode, NextTarget).

// +!find_and_recurse(LastNode)
//     <-
//     .print("No next segment found from ", LastNode).                    

