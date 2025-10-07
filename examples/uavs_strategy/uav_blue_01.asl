!start.

// +!start <-
//   .print("Blue 01 started").

// +!subtask_coord <-
//   ?subtask_done(SubTaskName);
//   .print("receive subtask state:", SubTaskName);
//   .print("start subtask02");
//   ?friend(Friend);
//   .send(Friend, tell, subtask_done("blue01_02_taskdone"));
//   .send(Friend, achieve, subtask_coord).

+!start <-
  .act_get_position(X);

  !ta1;
  ?my_friend(Friend);
  .print("Uav A Got friend: ", Friend);
  .wait(400);
  .send(Friend, tell, ta1_done("from_uav01"));
  .send(Friend, achieve, ta1_taskdone);
  .print("Uav A start-process done.").

+tb1_done(Msg) <- .print("tb1_done received: ", Msg).

+!tb1_taskdone <-
  .print("tb1_taskdone received, ready to execute t2");
  !t2;
  !ta3.

+!ta1 <- .act_detour(ring2); .print("ta1 executed").
+!t2 <- .act_attack(hq2); .print("t2 executed").
+!ta3 <- .act_escape(ring1); .print("ta3 executed").

// +tb1_done(b) <- .print("tb1_done received: ", b).
