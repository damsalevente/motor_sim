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

struct Controller{
    /* parameters */
    p: f32,
    i: f32,
    d: f32,
    /* errors */
    err_d: f32, /* previous error for d */
    err_i: f32, /* sum of errors for i */
    /* limits */
    minimum: f32,
    maximum: f32,
    integral_min: f32,
    integral_max: f32,

    /* target */
    target: f32,
}
impl Controller{
    fn new(p: f32, i: f32, d: f32, min: f32, M: f32) -> Controller {
        Controller {p:p,
        i:i,
        d:d,
        err_d: 0.0,
        err_i: 0.0,
        minimum: min,
        maximum: M,
        integral_min: min,
        integral_max: M,
        target: 0.0,
        }
    }
    /* call to set the target to control to */
    fn set_reference(&mut self, target: f32) {
        self.target = target;
    }

    /* get the input value for the desired target, it should be set beforehand */
    fn run(&mut self, u: f32) -> f32 {
        let err = self.target - u;
        let err_d = err - self.err_d;
        self.err_i += err;

        let mut ctrl_sig = self.p * err +  /* P */
                       self.i * self.err_i + /* I */
                       self.d * self.err_d;  /* D */

        if ctrl_sig < self.minimum {
            ctrl_sig = self.minimum;
        }
        else if ctrl_sig > self.maximum {
            ctrl_sig = self.maximum;
        }
        /*
        if self.err_i < self.integral_min {
            self.err_i = self.integral_min;
        }
        else if self.err_i > self.integral_max {
            self.err_i = self.integral_max;
        }
        */
        return ctrl_sig;
    }
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
    fn adaptrk4(&mut self, t: f32, ht: &mut f32, mut ht1: f32, eps: f32,y: &mut Vec<f32>, n: i32, func: fn(MotorParams, f32, &mut Vec<f32>) -> Vec<f32> , params: MotorParams) -> Vec<f32>
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
            ht2 = (*ht) / 2.0;
            yt2 = y.to_vec();
            yt_a = y.to_vec();
            self.rk4(t, *ht,&mut yt_a,n, func, params);
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
            if err != 0.0 {
                f = 0.9 * libm::powf(eps / err, 0.25);
            }
            if f > 5.0 {
                f = 5.0;
            }
            ht1 = f * (*ht);
            if err <= eps {
                break;
            }
            *ht = ht1;
        }
        *y = yt2;
        return y.to_vec();
    }

    fn rk4(&mut self, t: f32, ht: f32, y: &mut Vec<f32>, n: i32, func: fn(MotorParams, f32, &mut Vec<f32>) -> Vec<f32>, params: MotorParams)
    {
        let mut yt: Vec<f32> = vec![0.0;n as usize];
        let ht2 = ht / 2.0;
        let f1 = func(params, t, y);
        for i in (0..(n as usize)-1 as usize){
            yt[i] = y[i] + ht2 * f1[i];
        }

        let f2 = func(params, t + ht2, &mut yt);
        for i in (0..(n as usize)-1 as usize) {
            yt[i] = y[i] + ht2 * f2[i];
        }

        let f3 = func(params, t + ht, &mut yt);
        for i in (0..(n as usize)-1 as usize){
            yt[i] = y[i] + ht * f3[i];
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
        let ht1:f32 = 0.0001; /* todo */
        let mut time: f32 = t;
        while time + ht < next_t
        {
            res = self.adaptrk4(t, &mut ht, ht1, 1e-2, u, self.n as i32, func, params);
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
        self.theta %= 360.0;
    }
    fn set_voltages(&mut self, ud: f32, uq: f32){
        self.u[3] = ud;
        self.u[4] = uq;
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
        println!("----------------");
        println!("Time: {}", self.t);
        println!("ud:\t{}V\tuq:\t{}V", self.ud(), self.uq());
        println!("id:\t{}A\tiq:\t{}A", self.id(), self.iq());
        println!("Speed:\t{} rad/s", self.speed());
        println!("Theta:\t{} degree", self.theta());
        println!("-----------------");
    }
}



fn main() {
    let mut motor: Motor = Motor::default();

    let mut iq_ctrl: Controller = Controller::new(0.05, 0.01, 0.000, 3000.0, -3000.0);
    let mut id_ctrl: Controller  = Controller::new(1.5, 0.01, 0.000, 3000.0, -3000.0);
    let mut wr_controller: Controller = Controller::new(1.5, 0.01, 0.000, 30000.0, -30000.0);

    let target_speed = 5.0;
    let mut curr_cmd = 0.5;
    wr_controller.set_reference(target_speed);
    for i in 0..10{
        if i % 5 == 0 {
            curr_cmd = wr_controller.run(motor.speed()); 
        }
        id_ctrl.set_reference(0.0);
        iq_ctrl.set_reference(curr_cmd); 
        motor.set_voltages(id_ctrl.run(motor.id()), iq_ctrl.run(motor.iq()));
        motor.step(0.05);
        motor.print();
    }
}
