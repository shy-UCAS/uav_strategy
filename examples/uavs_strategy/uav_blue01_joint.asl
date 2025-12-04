// uav_blue_redis_01.asl 
+!merge_to_ring <- 
    .print("Merging to Global Ring, I am segment 1/3");
    // 调用 Python 动作更新状态
    .act_update_formation_state(
        true,   // is_merged
        3,      // total_clusters
        0,      // my_index (0, 1, 2)
        "ring", // type
        500     // radius
    ).

// uav_blue_redis_02.asl (我是2号)
+!merge_to_ring <- 
    .act_update_formation_state(true, 3, 1, "ring", 500).

// uav_blue_redis_03.asl (我是3号)
+!merge_to_ring <- 
    .act_update_formation_state(true, 3, 2, "ring", 500).