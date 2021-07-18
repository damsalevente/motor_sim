use libm::fabsf;

#[derive(Debug, Copy, Clone)]
struct MotorParams{
    R: f32,
    Ld: f32,
    Lq: f32,
    J: f32,
    P: f32,
    B: f32,
    lambda: f32
}

impl MotorParams{
    fn SmallMotor() -> MotorParams
    {
        MotorParams{
        R : 2.5,
        Ld : 67e-3,
        Lq : 67e-3,
        J : 5e-5,
        P : 4.0,
        B : 1.0,
        lambda : 0.17
        }
    }
    fn MediumMotor() -> MotorParams {
        MotorParams{R: 2.6,
        Ld: 6.7e-3,
        Lq: 6.7e-3,
        J: 3.5e-5,
        P: 3.0,
        B: 5e-5,
        lambda: 0.319}
    }
    fn RealMotor() -> MotorParams{
        MotorParams{
        R : 4.3,
        Ld : 67e-3,
        Lq : 27e-3,
        J : 3.5e-5,
        P : 4.0,
        B : 5e-5,
        lambda : 0.272,
        }
    }
}


struct Rk4AdaptSolver{
    x: f32, /* time */
    y: Vec<f32>, /* result */
    n: i8,
}

impl Rk4AdaptSolver{
    fn new(n: i8) -> Rk4AdaptSolver{
        Rk4AdaptSolver{
            x : 0.0,
            y: vec![0.0;n as usize],
            n: n,
        }
    }
    fn adaptrk4(&mut self, t: f32, mut ht: f32, mut ht1: f32, eps: f32,y: &mut Vec<f32>, n: i32, func: fn(MotorParams, f32, &mut Vec<f32>) -> Vec<f32> , params: MotorParams) -> Vec<f32>
    {
        let mut err: f32;
        let mut erri: f32;
        let mut yt2 = y.to_vec();
        let mut yt_a = y.to_vec();
        let mut f: f32;
        let mut ht2: f32;
        for iterations in 0..9 {
            if iterations >= 9{
                println!("max number of iterations exceeded!");
            }
            ht2 = ht / 2.0;
            yt2 = y.to_vec();
            yt_a = y.to_vec();
            self.rk4(t, ht,&mut yt_a,n, func, params);
            self.rk4(t, ht2,&mut yt2,n, func, params);
            self.rk4(t+ht2, ht2,&mut yt2,n, func, params);
            err = 0.0;
            for iter in 0..n-1{
                let i = iter as usize;
                if yt2[i] > 0.0 {
                    erri = fabsf(1.0 - yt_a[i] / yt2[i]);
                }
                else{
                    erri = fabsf(yt2[i] - yt_a[i]);
                }
                if err < erri {
                    err = erri;
                }
            }
            f = 1.0;
            if err > 0.0 {
                f = 0.9 * libm::powf(eps / err, 0.25);
            }
            if f > 5.0 {
                f = 5.0;
            }
            ht1 = f * ht;
            if err <= eps {
                break;
            }
            ht = ht1;
        }
        *y = yt2;
        return y.to_vec();
    }

    fn rk4(&mut self, t: f32, ht: f32, y: &mut Vec<f32>, n: i32, func: fn(MotorParams, f32, &mut Vec<f32>) -> Vec<f32>, params: MotorParams)
    {
        let mut yt: Vec<f32> = vec![0.0;n as usize];
        let ht2 = ht / 2.0;
        let f1 = func(params, t, y);
        for ((yt_i, y_i), f_i) in yt.iter_mut().zip(y.iter()).zip(f1.iter()) {
            *yt_i = y_i + ht2 * f_i;
        }

        let f2 = func(params, t + ht2, &mut yt);
        for ((yt_i, y_i), f_i) in yt.iter_mut().zip(y.iter()).zip(f2.iter()) {
            *yt_i = y_i + ht2 * (f_i);
        }

        let f3 = func(params, t + ht, &mut yt);
        for ((yt_i, &y_i), &f_i) in yt.iter_mut().zip(y.iter()).zip(f3.iter()) {
            *yt_i = y_i + ht * f_i;
        }

        let f4 = func(params,t + ht, &mut yt);
        let ht6 = ht / 6.0;
        for iter in 0..n-1{
            let i = iter as usize;
            y[i] += ht6 * (f1[i] + 2.0 * (f2[i] + f3[i]) + f4[i]);
        }

    }
}

trait Solver{
    fn init(&mut self, n: usize);
    fn step(&mut self,u: &mut Vec<f32>, t: f32, next_t: f32, func: fn(MotorParams, f32, &mut Vec<f32>) -> Vec<f32>, params: MotorParams) -> Vec<f32>;
}

impl Solver for Rk4AdaptSolver{

    fn init(&mut self, n: usize)
    {
        self.x = 0.0;
        self.y = vec![0.0; n];
        self.n = n as i8;
    }
    fn step(self: &mut Rk4AdaptSolver,u: &mut Vec<f32>, t:f32, next_t: f32, func: fn(MotorParams, f32, &mut Vec<f32>) -> Vec<f32>,params: MotorParams) -> Vec<f32>
    {
        let mut ht:f32 = 0.001; /* todo */
        let mut res: Vec<f32> = u.to_vec();
        let ht1:f32 = 0.01; /* todo */
        let mut time: f32 = t;
        while time + ht < next_t
        {
            ht = ht1;
            res = self.adaptrk4(t, ht, ht1, 1e-2, u, self.n as i32, func, params);
            time += ht;
        }
    return res;
    }
}


fn pmsm_equation(params: MotorParams, t: f32, u: &mut Vec<f32>) -> Vec<f32>
{
    let mut result: Vec<f32> = vec![0.0;6];
    result[0] = -params.R / params.Ld * u[0] + params.Lq / params.Ld * params.P * u[2] * u[1] + 1.0/params.Ld * u[3];
    result[1] = -params.R/ params.Lq * u[1] - params.Ld / params.Lq * params.P * u[3] * u[0] - (params.lambda * params.P * u[2]) / params.Lq /params.Lq + 1.0 / params.Lq * u[4]; 
    result[2] = params.P / params.J * (params.lambda * u[1] + (params.Ld - params.Lq) * u[1] * u[0]) - params.B / params.J * u[3] - u[5] / params.J;
    result[3] = 0.0;
    result[4] = 0.0;
    result[5] = 0.0;
    return result;
}

struct Motor
{
    params: MotorParams,
    solver: Box<dyn Solver>,
    t: f32,
    torque: f32,
    theta: f32,
    u: Vec<f32>,
}
impl Motor
{
    fn default() -> Motor{
        Motor{
            params: MotorParams::SmallMotor(),
            solver: Box::new(Rk4AdaptSolver::new(6)),
            t: 0.0,
            torque: 0.0,
            theta: 0.0,
            u: vec![0.0; 6]
        }
    }
    fn step(&mut self,dt: f32)
    {
        self.solver.step(&mut self.u, self.t, self.t+dt, pmsm_equation, self.params);
        self.calc_torque();
        self.calc_theta();
        self.t += dt;
    }

    fn calc_torque(&mut self) {
        self.torque = 3.0 * self.params.P/4.0 *((self.params.Ld - self.params.Lq)*self.id() * self.iq() + self.params.lambda* self.id());
    }

    fn calc_theta(&mut self) {
        self.theta += self.speed();
        if self.theta > 360.0
        {
            self.theta -= 360.0;
        }
        if self.theta < -360.0
        {
            self.theta += 360.0;
        }
    }
    fn speed(&self) -> f32{
        return self.u[2];
    }
    fn id(&self) ->f32 {
        return self.u[0];
    }
    fn iq(&self) ->f32 {
        return self.u[1];
    }
    fn ud(&self) ->f32 {
        return self.u[3];
    }
    fn uq(&self) ->f32 {
        return self.u[4];
    }
    fn ti(&self) -> f32 {
        return self.u[5];
    }
    fn theta(&self) -> f32 {
        return self.theta;
    }
    fn torque(&self) -> f32{
        return self.torque;
    }
    fn print(&self)
    {
        println!("ud:\t{}V\tuq:\t{}V", self.ud(), self.uq());
        println!("id:\t{}A\tiq:\t{}A", self.id(), self.iq());
        println!("Speed:\t{} rad/s", self.speed());
        println!("Theta:\t{} degree", self.theta());
    }
}



fn main() {
    let mut motor: Motor = Motor::default();
    motor.u[3] = 0.0;
    motor.u[4] = 200.0;
    for i in 0..100{
        motor.step(0.1);
        motor.print();
    }
}
