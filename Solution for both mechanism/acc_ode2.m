% For ode 45 with slider crank system

function acc = acc_ode2(M, F_dyn, Cq_fun_dyn, g_cap, t, q)

acc = [q(13:24) ; inv(M)*Cq_fun_dyn(t,q(1:12))'*inv((Cq_fun_dyn(t,q(1:12))*inv(M)*Cq_fun_dyn(t,q(1:12))'))*(g_cap(t,q(1:12),q(13:24))-Cq_fun_dyn(t, q(1:12))*inv(M)*F_dyn(q(1:12))) + inv(M)*F_dyn(q(1:12))];
end