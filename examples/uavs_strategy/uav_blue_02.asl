!start.

// +!start <-
//   .print("Blue 02 started");
//   ?friend(Friend);
//   .send(Friend, tell, subtask_done("blue02_01_taskdone"));
//   .send(Friend, achieve, subtask_coord).

// +!subtask_coord <-
//   ?subtask_done(SubTaskName);
//   .print("Blue 02 received subtask state:", SubTaskName);
//   .print("start subtask 02").

+!start <-
  !tb1;
  ?my_friend(Friend);
  .print("Uav B Got friend: ", Friend);
  .wait(400);
  .send(Friend, tell, tb1_done("from_uav02"));
  .send(Friend, achieve, tb1_taskdone);
  .print("Uav B start-process done.").

+ta1_done(Msg) <- .print("ta1_done received: ", Msg).

+!ta1_taskdone <-
  .print("ta1_taskdone received, ready to execute t2");
  !t2;
  !tb3.

+!tb1 <- .act_breakthrough(antiair); .print("tb1 executed").
+!t2 <- .act_escape(antiair_facilities); .print("t2 executed").
+!tb3 <- .act_attack(headquarter); .print("tb3 executed").

// +!ta1_done(a) <- .print("ta1_done received: ", a).
