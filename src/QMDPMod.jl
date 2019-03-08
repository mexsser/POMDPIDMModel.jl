function solve(solver::QMDPSolver, pomdp::POMDP; sparse::Bool=false)
    if sparse
        vi_solver = SparseValueIterationSolver(max_iterations=solver.max_iterations, belres=solver.tolerance, verbose=solver.verbose, include_Q=true)
        vi_policy = solve(vi_solver, pomdp)
    else
        vi_solver = ValueIterationSolver(max_iterations=solver.max_iterations, belres=solver.tolerance, verbose=solver.verbose, include_Q=true)
        vi_policy = solve(vi_solver, pomdp)
    end

    return AlphaVectorPolicy(pomdp, vi_policy.qmat)
end
