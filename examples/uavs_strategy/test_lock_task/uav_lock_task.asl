
can_task_start(true).
!start.

+!start
<-
    .print("UAV lock task started");
    !task_breakthrough;
    !task_detour.


+!task_breakthrough : can_task_start(true)
<-
    ?can_task_start(IF);
    .print("start breakthrough task ...",IF);

    -can_task_start(true);
    +can_task_start(false);

    .print("breakthrough planned, waiting to be executed").




+!task_detour : can_task_start(true)
<-
    .print("start detour task ...");
    -+can_task_start(false);
    .print("detour planned, waiting to be executed").


+!task_detour : not can_task_start(true)
<-
    .wait(100);
    ?can_task_start(IF);
    .print("task detour possible, waiting to be executed",IF);


    !task_detour.



