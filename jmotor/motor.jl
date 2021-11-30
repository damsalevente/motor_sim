using Printf
using DynamicalSystems 

@enum Params::Int ID = 1 IQ = 2 WR = 3 VD = 4 VQ = 5 TI = 6 THETA = 7 # easier array indexing
Base.to_index(p::Params) = Int(p) # convert the enums to int to index

struct Motor
  R
  Ld
  Lq
  J
  P
  B
  lambda
end

@inline @inbounds function motor_func(F, u, m, t)
  # F: dy/dx, u: x, m: motor parameters, t: time  
  F[1] = -m.R / m.Ld * u[ID] + m.Lq / m.Ld * m.P * u[WR] * u[IQ] + 1 / m.Ld * u[VD]
  F[2] = -m.R / m.Lq * u[IQ] - m.Ld / m.Lq * m.P * u[WR] * u[ID] - (m.lambda * m.P * u[WR]) / m.Lq + 1 / m.Lq * u[VQ]
  F[3] = m.P / m.J * (m.lambda * u[IQ] + (m.Ld - m.Lq) * u[IQ] * u[ID]) - m.B / m.J * u[WR] - u[TI] / m.J
  F[4] = 0.0
  F[5] = 0.0
  F[6] = 0.0
  F[7] = u[WR]
end

motor = Motor(2.5, 67e-3, 67e-3, 5e-5, 4, 1, 0.17)

u = zeros(7)
u[VD] = 10.0
u[VQ] = 10.0

ds = ContinuousDynamicalSystem(motor_func, u, motor)

dataset = trajectory(ds::DynamicalSystem, 10; Δt = 0.01)

for point in dataset
  println(point[3])
end