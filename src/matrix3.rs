use std::ops::Mul;
use std::convert::From;
use super::quaternion::Quat;
use super::vector::Vec3;

#[derive(Debug, Clone, Copy)]
pub struct Mat3 {
        pub v: [f64; 9]
}

impl From<Quat> for Mat3 {
    fn from(o: Quat) -> Mat3 {
        let v = [
            o.w().powi(2) + o.x().powi(2) - o.y().powi(2) - o.z().powi(2),
            2.0 * (o.x() * o.y() - o.w() * o.z()),
            2.0 * (o.x() * o.z() + o.w() * o.y()),
            2.0 * (o.x() * o.y() + o.w() * o.z()),
            o.w().powi(2) - o.x().powi(2) + o.y().powi(2) - o.z().powi(2),
            2.0 * (o.y() * o.z() - o.w() * o.x()),
            2.0 * (o.x() * o.z() - o.w() * o.y()),
            2.0 * (o.y() * o.z() + o.w() * o.x()),
            o.w().powi(2) - o.x().powi(2) - o.y().powi(2) + o.z().powi(2)
        ];

        Mat3{v}
    }
}

impl Mul for Mat3 {
        type Output = Mat3;

        fn mul(self, other: Mat3) -> Mat3 {
                let mut r = Mat3::zero();

                r.v[0] = self.m00() * other.m00()
                        + self.m01() * other.m10()
                        + self.m02() * other.m20();

                r.v[1] = self.m00() * other.m01()
                        + self.m01() * other.m11()
                        + self.m02() * other.m21();

                r.v[2] = self.m00() * other.m02()
                        + self.m01() * other.m12()
                       + self.m02() * other.m22();

                r.v[3] = self.m10() * other.m00()
                        + self.m11() * other.m10()
                        + self.m12() * other.m20();

                r.v[4] = self.m10() * other.m01()
                        + self.m11() * other.m11()
                        + self.m12() * other.m21();

                r.v[5] = self.m10() * other.m02()
                        + self.m11() * other.m12()
                        + self.m12() * other.m22();

                r.v[6] = self.m20() * other.m00()
                        + self.m21() * other.m10()
                        + self.m22() * other.m20();

                r.v[7] = self.m20() * other.m01()
                        + self.m21() * other.m11()
                        + self.m22() * other.m21();

                r.v[8] = self.m20() * other.m02()
                        + self.m21() * other.m12()
                        + self.m22() * other.m22();
                r
        }
}

impl Mat3 {
        pub fn zero() -> Mat3 {
                let v = [
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0 ];               
                Mat3 { v }
        }

        pub fn new_from_angle_axis(rad: f64, axis: Vec3) -> Mat3 {
            let c = rad.cos();
            let s = rad.sin();
            let a = axis.unitize();
            let x = a.x();
            let y = a.y();
            let z = a.z();

            let v = [
                c + (1.0 - c) * x.powi(2),
                (1.0 - c) * x * y - s * z,
                (1.0 - c) * x * z + s * y,
                (1.0 - c) * x * y + s * z,
                c + (1.0 - c) * y.powi(2),
                (1.0 - c) * y * z - s * x,
                (1.0 - c) * x * z - s * y,
                (1.0 - c) * y * z + s * x,
                c + (1.0 - c) * z.powi(2)
            ];
            Mat3{v}
        }

        pub fn yaw_pitch_roll(&self) -> (f64, f64, f64) {
            // z -> y -> x
            let beta =  (-self.m20()).asin(); // pitch
            let gamma = (self.m21() / beta.cos()).asin();// yaw
            let alpha = (self.m00() / beta.cos()).acos();// roll

            (gamma, beta, alpha)
        }

        pub fn eular_angle_zyz(&self) -> (f64, f64, f64) {
            let theta = self.m22().acos(); // no.2
            let sin_theta = theta.sin();
            let phi = (self.m12()/sin_theta).asin(); // no.1
            let psi = (self.m21()/sin_theta).asin(); // no.3

            (phi, theta, psi)
        }

        //FIXME: impl for fmt
        #[allow(dead_code)]
        pub fn show(&self) {
                println!(
                        "
            {}, {}, {}, \n
            {}, {}, {}, \n
            {}, {}, {} 
        ",
                        self.v[0],
                        self.v[1],
                        self.v[2],
                        self.v[3],
                        self.v[4],
                        self.v[5],
                        self.v[6],
                        self.v[7],
                        self.v[8]
                );
        }

        pub fn m00(&self) -> f64 {
                self.v[0]
        }
        pub fn m01(&self) -> f64 {
                self.v[1]
        }
        pub fn m02(&self) -> f64 {
                self.v[2]
        }
        pub fn m10(&self) -> f64 {
                self.v[3]
        }
        pub fn m11(&self) -> f64 {
                self.v[4]
        }
        pub fn m12(&self) -> f64 {
                self.v[5]
        }
        pub fn m20(&self) -> f64 {
                self.v[6]
        }
        pub fn m21(&self) -> f64 {
                self.v[7]
        }
        pub fn m22(&self) -> f64 {
                self.v[8]
        }
}
