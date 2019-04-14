function acc = system_accelerations(t, q, qp, M, sforce, grav, bodies, Cq_fun_dyn, g)

F = force_vector(grav, sforce, bodies, q);

acc = inv(M) * Cq_fun_dyn' * inv(Cq_fun_dyn * inv(M) * Cq_fun_dyn') * (g - Cq_fun_dyn * inv(M) * F) + inv(M)*F;
