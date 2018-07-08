use std::ops::Mul;

pub struct Mat4 {
        pub v: [f64; 16],
}

impl Mul for Mat4 {
        type Output = Mat4;

        fn mul(self, other: Mat4) -> Mat4 {
                let mut r = Mat4::zero();

                r.v[0] = self.m00() * other.m00()
                        + self.m01() * other.m10()
                        + self.m02() * other.m20()
                        + self.m03() * other.m30();

                r.v[1] = self.m00() * other.m01()
                        + self.m01() * other.m11()
                        + self.m02() * other.m21()
                        + self.m03() * other.m31();

                r.v[2] = self.m00() * other.m02()
                        + self.m01() * other.m12()
                        + self.m02() * other.m22()
                        + self.m03() * other.m32();

                r.v[3] = self.m00() * other.m03()
                        + self.m01() * other.m13()
                        + self.m02() * other.m23()
                        + self.m03() * other.m33();

                r.v[4] = self.m10() * other.m00()
                        + self.m11() * other.m10()
                        + self.m12() * other.m20()
                        + self.m13() * other.m30();

                r.v[5] = self.m10() * other.m01()
                        + self.m11() * other.m11()
                        + self.m12() * other.m21()
                        + self.m13() * other.m31();

                r.v[6] = self.m10() * other.m02()
                        + self.m11() * other.m12()
                        + self.m12() * other.m22()
                        + self.m13() * other.m32();

                r.v[7] = self.m10() * other.m03()
                        + self.m11() * other.m13()
                        + self.m12() * other.m23()
                        + self.m13() * other.m33();

                r.v[8] = self.m20() * other.m00()
                        + self.m21() * other.m10()
                        + self.m22() * other.m20()
                        + self.m23() * other.m30();

                r.v[9] = self.m20() * other.m01()
                        + self.m21() * other.m11()
                        + self.m22() * other.m21()
                        + self.m23() * other.m31();

                r.v[10] = self.m20() * other.m02()
                        + self.m21() * other.m12()
                        + self.m22() * other.m22()
                        + self.m23() * other.m32();

                r.v[11] = self.m20() * other.m03()
                        + self.m21() * other.m13()
                        + self.m22() * other.m23()
                        + self.m23() * other.m33();

                r.v[12] = self.m30() * other.m00()
                        + self.m31() * other.m10()
                        + self.m32() * other.m20()
                        + self.m33() * other.m30();

                r.v[13] = self.m30() * other.m01()
                        + self.m31() * other.m11()
                        + self.m32() * other.m21()
                        + self.m33() * other.m31();

                r.v[14] = self.m30() * other.m02()
                        + self.m31() * other.m12()
                        + self.m32() * other.m22()
                        + self.m33() * other.m32();

                r.v[15] = self.m30() * other.m03()
                        + self.m31() * other.m13()
                        + self.m32() * other.m23()
                        + self.m33() * other.m33();

                r
        }
}

impl Mat4 {
        // fn identity() -> Mat4 {
        //         let v = [
        //                 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        //                 1.0,
        //         ];
        //         Mat4 { v }
        // }

        pub fn zero() -> Mat4 {
                let v = [
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0,
                ];
                Mat4 { v }
        }

        //FIXME: impl for fmt
        #[allow(dead_code)]
        pub fn show(&self) {
                println!(
                        "
            {}, {}, {}, {}, \n
            {}, {}, {}, {}, \n
            {}, {}, {}, {}, \n
            {}, {}, {}, {}
        ",
                        self.v[0],
                        self.v[1],
                        self.v[2],
                        self.v[3],
                        self.v[4],
                        self.v[5],
                        self.v[6],
                        self.v[7],
                        self.v[8],
                        self.v[9],
                        self.v[10],
                        self.v[11],
                        self.v[12],
                        self.v[13],
                        self.v[14],
                        self.v[15]
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
        pub fn m03(&self) -> f64 {
                self.v[3]
        }
        pub fn m10(&self) -> f64 {
                self.v[4]
        }
        pub fn m11(&self) -> f64 {
                self.v[5]
        }
        pub fn m12(&self) -> f64 {
                self.v[6]
        }
        pub fn m13(&self) -> f64 {
                self.v[7]
        }
        pub fn m20(&self) -> f64 {
                self.v[8]
        }
        pub fn m21(&self) -> f64 {
                self.v[9]
        }
        pub fn m22(&self) -> f64 {
                self.v[10]
        }
        pub fn m23(&self) -> f64 {
                self.v[11]
        }
        pub fn m30(&self) -> f64 {
                self.v[12]
        }
        pub fn m31(&self) -> f64 {
                self.v[13]
        }
        pub fn m32(&self) -> f64 {
                self.v[14]
        }
        pub fn m33(&self) -> f64 {
                self.v[15]
        }
}
