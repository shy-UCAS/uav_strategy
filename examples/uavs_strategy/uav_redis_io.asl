!start.

+!start
<-
    ?start_height(START_HEIGHT);
    .print("start height is:",START_HEIGHT);
    .act_breakthrough(headquarter,START_HEIGHT,-1);
    ?if_set_ref_traj(IF_SET);
    .print("whether need to set ref traj",IF_SET);
    .print("breakthrough executed").
