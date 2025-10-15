!start.

+!start <-
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
  !ta3;
  !visualize.

+!ta1 <- .act_detour(probe_facilities,-1,-1); .print("ta1 executed").
+!t2 <- .act_attack(headquarter); .print("t2 executed").
+!ta3 <- .act_breakthrough(antiair,-1,-1); .print("ta3 executed").
+!visualize <-  .print("ta4 executed").

