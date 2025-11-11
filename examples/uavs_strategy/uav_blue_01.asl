!start.

task_sequence([task1, task2, task3]).

+!start <-
  !ta1;
  ?my_friend(Friend);
  .print("Uav A Got friend: ", Friend);
  .wait(400);
  .send(Friend, tell, ta1_done("from_uav01"));
  .send(Friend, achieve, ta1_taskdone);
  .print("Uav A start-process done.").

+!start <-
  !task_section01.

+!task_section01 <-
  .act_breakthrough(antiair,-1,-1);
  .check_join_status([blue02, blue03], section02_start).

+section02_start <-
  !task_section02.

+!task_section02: condition <-
  .act_xxx.

+tb1_done(Msg) <- .print("tb1_done received: ", Msg).

// +red_alert(Msg) <- .act_escape_from_red.
// +blue_avoid(Msg) <- .act_avoid_blue.

+!tb1_taskdone <-
  .print("tb1_taskdone received, ready to execute t2");
  !t2;
  !ta3;
  !visualize.

+!ta1 <- .act_detour(probe_facilities,-1,-1); .print("ta1 executed").
+!t2 <- .act_attack(headquarter); .print("t2 executed").
+!ta3 <- .act_breakthrough(antiair,-1,-1); .print("ta3 executed").
+!visualize <-  .print("ta4 executed").

+!avoid_red_enemy <- .act_escape_from_red; 
+!avoid_blue_enemy <- .act_avoid_blue.