use libm::fabsf;

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
    func: fn(f32, &mut Vec<f32>) -> Vec<f32> /* rhs equation */
}

impl Rk4AdaptSolver{
    fn adaptrk4(&mut self, t: f32, ht: f32, ht1: f32, eps: f32,y: &mut Vec<f32>, n: i32)
    {
        let mut err: f32;
        let mut erri: f32;
        let mut f: f32;
        let mut ht2: f32;
        let mut yt2 = y;
        for iterations in 0..9 {
            if iterations >= 9{
                println!("max number of iterations exceeded!");
            }
            ht2 = ht / 2.0;
            let mut yt_a = y;
            self.rk4(t, ht, yt_a,n);
            self.rk4(t, ht2, yt_a,n);
            self.rk4(t+ht2, ht2, yt2,n);
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
        self.y = *yt2;
    }
    fn rk4(&mut self, t: f32, ht: f32, y: &mut Vec<f32>, n: i32)
    {

        let mut yt: Vec<f32> = Vec::with_capacity(n as usize);
        let ht2 = ht / 2.0;
        let f1 = (self.func)(t, y);
        for ((&yt_i, &y_i), &f_i) in yt.iter().zip(y.iter()).zip(f1.iter()) {
            yt_i = y_i + ht2 * f_i;
        }

        let f2 = (self.func)(t + ht2, &mut yt);
        for ((&yt_i, &y_i), &f_i) in yt.iter().zip(y.iter()).zip(f2.iter()) {
            yt_i = y_i + ht2 * f_i;
        }

        let f3 = (self.func)(t + ht, &mut yt);
        for ((&yt_i, &y_i), &f_i) in yt.iter().zip(y.iter()).zip(f3.iter()) {
            yt_i = y_i + ht * f_i;
        }

        let f4 = (self.func)(t + ht, &mut yt);
        let ht6 = ht / 6.0;
        for iter in 0..n-1{
            let mut i = iter as usize;
            y[i] += ht6 * (f1[i] + 2.0 * (f2[i] + f3[i]) + f4[i]);
        }
        self.x = self.x + ht;
        self.y = y.to_vec();
    }
}

trait Solver{
    fn init(&self, eq: fn(Vec<f32>), n: usize);
    fn step(&self,u: &mut Vec<f32>, t: f32, next_t: f32) -> Vec<f32>;
}

impl Solver for Rk4AdaptSolver{

    fn init(&self, eq: fn(Vec<f32>), n: usize)
    {
        self.x = 0.0;
        self.y = Vec::with_capacity(n as usize);
        self.n = n as i8;
    }
    fn step(&self,u: &mut Vec<f32>, t:f32, next_t: f32) -> Vec<f32>
    {
        let mut ht:f32 = 0.01; /* todo */
        let mut ht1:f32 = 0.01; /* todo */
        while t + ht < next_t
        {
            ht = ht1;
            self.adaptrk4(t, ht, ht1, 1e-3, u, self.n as i32);
        }
        return self.y
    }
}

struct Motor
{
    params: MotorParams,
    solver: Box<dyn Solver>,
    id: f32, /* id current */
    iq: f32, /* iq current */
    ud: f32, /* ud voltage */
    uq: f32, /* uq voltage */
    theta: f32, /* rotor angle */
    ti: f32, /* terheles */
    speed: f32, /* angular speed in rad/s */
    torque: f32,
}

impl Motor()
{
    fn basic() -> Motor{
        Motor{
            params: MotorParams::SmallMotor(),
            solver: Box::new(Rk4AdaptSolver()),
            id: 0.0,
            iq: 0.0,
            ud: 0.0,
            uq: 0.0,
            theta: 0.0,
            speed: 0.0,
            torque: 0.0,
        }
    }
}



fn main() {
    Motor motor(Motor::smallMotor);
    motor.start();
    motor.step();
    motor.ud = 100;
    motor.uq = 200;
    motor.step();
    motor.currents(); /* tuple */
    motor.voltages() /* tuple */
    motor.speed;
    println!("Hello, world!");
}
