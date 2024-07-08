import torch


def get_parameters(args):
    if args.tune_mppi <= 0:
        args.α = 2.62e-1
        args.λ = 1.60e1
        args.σ = 4.52e1
        args.χ = 2.00e-2
        args.ω1 = 10.37
        args.ω2 = 9.26e3
        args.ω_Φ = 5.41

    K = args.mppi_k
    T = args.mppi_t
    Δt = 0.5  # Real Environment 0.5s Steptime 0.01

    dtype = torch.double
    device = args.device

    α = args.α
    λ = args.λ
    Σ = args.σ * torch.tensor([
        [1, args.χ, args.χ],
        [args.χ, 1, args.χ],
        [args.χ, args.χ, 1]
    ], dtype=dtype, device=device)

    def dynamics(x, u):
        u[:, 2] = 0
        new_pos = x[:, 0:3] + x[:, 3:6] * Δt
        new_pos[:, 0] = torch.clamp(new_pos[:, 0], 0.8, 1.5)
        new_vel = x[:, 3:6] + u * Δt
        return torch.cat((new_pos, new_vel), dim=1)

    def state_cost(x, goal, obstacles):
        cost = 1000 * torch.norm((x[:, 0:3] - goal), dim=1) ** 2
        cost -= args.ω1 * torch.norm(x[:, 3:6], dim=1) ** 2

        dist1 = x[:, 0:3] - obstacles[0:3]
        dist2 = x[:, 0:3] - obstacles[6:9]
        collision = torch.logical_or(torch.all(torch.le(dist1, torch.tensor(obstacles[3:6]) + 0.03), dim=1),
                                     torch.all(torch.le(dist2, torch.tensor(obstacles[9:12]) + 0.03), dim=1))
        cost += args.ω2 * collision

        return cost

    def terminal_cost(x, goal):
        cost = 10 * torch.norm((x[:, 0:3] - goal), dim=1) ** 2
        cost += args.ω_Φ * torch.norm(x[:, 3:6], dim=1) ** 2
        return cost

    def convert_to_target(x, u):
        return x[0:3] + u * Δt

    return K, T, Δt, α, dynamics, state_cost, terminal_cost, Σ, λ, convert_to_target, dtype, device
