!start.

+!start
<-
    ?start_height(START_HEIGHT);
    .print("start height is:",START_HEIGHT);
    .act_detour(probe_facilities,START_HEIGHT,-1);
    -if_set_ref_traj(_).
    +if_set_ref_traj(True).
    .print("whether need to set ref traj",X);
    .print("detour executed").
