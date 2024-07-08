import torch


def get_parameters(args):
    if args.tune_mppi <= 0:
        args.α = 0.400
        args.λ = 13.0
        args.σ = 45.2
        args.χ = 0.02
        args.ω1 = 5.30
        args.ω_Φ = 5.41

    K = args.mppi_k
    T = args.mppi_t
    Δt = 0.01

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
        return cost

    def terminal_cost(x, goal):
        cost = 10 * torch.norm((x[:, 0:3] - goal), dim=1) ** 2
        cost += args.ω_Φ * torch.norm(x[:, 3:6], dim=1) ** 2
        return cost

    def convert_to_target(x, u):
        return x[0:3] + u * Δt

    return K, T, Δt, α, dynamics, state_cost, terminal_cost, Σ, λ, convert_to_target, dtype, device
