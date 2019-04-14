% For ode 45 with four bar linkage

function acc = acc_ode(M, F_dyn, Cq_fun_dyn, g_cap, t, q)

acc = [q(16:30) ; inv(M)*Cq_fun_dyn(t,q(1:15))'*inv((Cq_fun_dyn(t,q(1:15))*inv(M)*Cq_fun_dyn(t,q(1:15))'))*(g_cap(t,q(1:15),q(16:30))-Cq_fun_dyn(t, q(1:15))*inv(M)*F_dyn(q(1:15))) + inv(M)*F_dyn(q(1:15))];
end
