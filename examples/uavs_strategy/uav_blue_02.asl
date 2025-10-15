!start.

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
  !visual.

+!tb1 <- .act_breakthrough(antiair,-1,-1); .print("tb1 executed").
+!t2 <- .act_escape(antiair_facilities,-1,-1); .print("t2 executed").
+!tb3 <- .act_detour(defence_rings,-1,-1); .print("tb3 executed").
+!visual <-  .print("blue 02 visual executed").

