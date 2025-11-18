// !start.

// +!start
// <-
//     ?start_height(START_HEIGHT);
//     .print("start height is:",START_HEIGHT);
//     .act_breakthrough(antiair,START_HEIGHT,-1);
//     -if_set_ref_traj(_);
//     +if_set_ref_traj(true);
//     .print("whether need to set ref traj",X);
//     .print("breakthrough executed").

can_task_start(true).
!start.

+!start
<-
    ?start_height(START_HEIGHT);
    .print("start height is:", START_HEIGHT);
    !task_breakthrough(START_HEIGHT);
    !task_detour.


+!task_breakthrough(START_HEIGHT) : can_task_start(true)
<-
    ?can_task_start(IF);
    .print("start breakthrough task ...", IF);

    -can_task_start(true);
    +can_task_start(false);

    .act_breakthrough(antiair, START_HEIGHT, -1);
    -+if_set_ref_traj(true);
    .print("breakthrough planned, waiting").


+!task_detour : can_task_start(true)
<-
    ?can_task_start(IF);
    .print("start detour task ...", IF);

    -can_task_start(true);
    +can_task_start(false);

    .act_escape(probe_facilities, -1, -1);
    -+if_set_ref_traj(true);
    .print("detour planned, waiting").


+!task_detour : not can_task_start(true)
<-
    .wait(100);
    ?can_task_start(IF);
    .print("detour waiting ...", IF);
    !task_detour.
