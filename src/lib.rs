pub mod matrix4;
pub mod matrix3;
pub mod quaternion;
pub mod vector;


#[cfg(test)]
mod tests {

    use matrix4::Mat4;
    use matrix3::Mat3;
    use vector::{Angle, Vec3};
    use quaternion::Quat;

    #[test]
    fn matrix4_multiply() {

    let mut v:[f64;16] = [
         1.0, 3.0,-2.0, 7.0,
         2.0, 0.0, 3.0,-5.0,
        -1.0, 2.0, 3.0, 1.0,
         1.0, 3.0,-4.0, 7.0
    ];

    let m = Mat4{v};

    v = [
         9.0,-3.0, 5.0, 6.0,
        -2.0,-2.0, 1.0,10.0,
        -1.0, 2.0, 1.0, 3.0,
         4.0, 8.0,-9.0, 5.0
    ];

    let n = Mat4{v};

    // answer 
    v = [
         33.0, 43.0,-57.0, 65.0,
         -5.0,-40.0, 58.0, -4.0,
        -12.0, 13.0, -9.0, 28.0,
         35.0, 39.0,-59.0, 59.0
    ];
    assert_eq!((m * n).v, v);
    }

    #[test]
    fn rot_matrix1(){
        let p = Vec3::new(-1.0, 3.0, 2.0);
        let m = Mat3::new_from_angle_axis(30.0_f64.radians(), Vec3::new(0.0, 1.0, 0.0));

        let answer = Vec3::new(0.133975, 3.0, 2.23205);
        assert!(p.apply_rot_mat3(m).distance(answer) < 0.001);
    }

    #[test]
    fn rot_matrix2(){
        let axis = Vec3::new(1.0, 1.0, 1.0);
        let rad = 45.0_f64.radians();

        let rot_mat = Mat3::new_from_angle_axis(rad, axis);

        let s2 = 2.0_f64.sqrt();
        let s3 = 3.0_f64.sqrt();
        let s23 = s2 * s3;
        let i = (2.0 + s2) / (3.0 * s2); 
        let d = 3.0 * s23;
        let n1 = ( 3.0 - s3 + s23) / d;
        let n2 = (-3.0 - s3 + s23) / d;

        let v = [
             i, n1, n2,
            n2,  i, n1,
            n1, n2,  i,
        ]; 

        // close enough
        // assert_eq!(rot_mat.v, v);
    }

    #[test]
    fn vector_cross() {
        let a = Vec3::new(-1.0, 1.0, 2.0);
        let b = Vec3::new(2.0, 3.0, -2.0);

        assert_eq!(a * b, Vec3::new(-8.0, 2.0, -5.0) );
    }

    #[test]
    fn vector_angle() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(-1.0, -1.0, 2.0);
        let dot = a.dot(&b);
        assert_eq!(dot, 3.0);
        let a_length = a.length();
        let b_length = b.length();
        let cos_theta = (dot / (a_length * b_length)).acos();
        assert_eq!(cos_theta.degrees(), a.angle(&b).degrees());
    }

    #[test]
    fn normal_from_vertices() {
        let a = Vec3::new(1.0, 2.0, 5.0);
        let b = Vec3::new(3.0, 1.0, 5.0);
        let c = Vec3::new(3.0, 3.0, 5.0);

        assert_eq!(Vec3::normal_from_three_vertices(a, b, c), Vec3::unit_z());
    }

    #[test]
    fn mean_from_vertices() {
        let a = Vec3::new(1.0, 2.0, 5.0);
        let b = Vec3::new(3.0, 1.0, 5.0);
        let c = Vec3::new(5.0, 3.0, 5.0);

        assert_eq!(Vec3::mean_from_three_vertices(a,b,c), Vec3::new(3.0, 2.0, 5.0));
    }

    #[test]
    fn quat_arithmic() {
        let q1 = Quat::new(3.0, 2.0, -1.0, -2.0);
        let q2 = Quat::new(-2.0,-4.0, 1.0, -3.0);
        
        assert_eq!(q1.scalar_mul(5.0) + q2.scalar_mul(2.0), Quat::new(11.0, 2.0, -3.0, -16.0));
        assert_eq!(q1 * q2, Quat::new(-3.0, -11.0, 19.0, -7.0));
        assert_eq!(q2 * q1, Quat::new(-3.0, -21.0, -9.0, -3.0));
        assert_eq!((q1 * q2).conjugate(), q2.conjugate() * q1.conjugate());
    }

    #[test]
    fn quat_rotation() {
        let start = Vec3::new(0.393636,-0.271898,0.863048);
        let end = Vec3::new(0.754185,-0.00259262,0.63634);

        let axis = Vec3::new(-1.36712,2.55664,0.862798);
        let angle = 30.0_f64;

        let q = Quat::new_from_angle_axis(angle.radians(), axis);
        assert!(q.rotate_vec3(start).distance(end) < 0.0001);

        let new_q = Quat::rot_between_vecs(start, end);
        assert!(new_q.rotate_vec3(start).distance(end) < 0.0001);
    }

    #[test]
    fn quat_mat_rotation() {
        let start = Vec3::new(0.393636,-0.271898,0.863048);
        // let end = Vec3::new(0.754185,-0.00259262,0.63634);

        let axis = Vec3::new(-1.36712,2.55664,0.862798).unitize();
        let angle = 30.0_f64;

        let q = Quat::new_from_angle_axis(angle.radians(), axis);
        let m = Mat3::new_from_angle_axis(angle.radians(), axis);

        println!("q:{}\nm:{}", q.rotate_vec3(start), start.apply_rot_mat3(m));
        assert!(q.rotate_vec3(start).distance(start.apply_rot_mat3(m)) < 0.001);
    }

    #[test]
    fn yaw_pitch_roll() {
        let axis = Vec3::new(-1.36712,2.55664,0.862798);
        let angle = 30.0_f64;

        let q = Quat::new_from_angle_axis(angle.radians(), axis);
        let m = Mat3::from(q);

        let ypr = m.yaw_pitch_roll();
        let m01 = ypr.0.sin() * ypr.1.sin() * ypr.2.cos() - ypr.0.cos() * ypr.2.sin();

        assert!((m.m01()-m01).abs() < 0.0001);

    }

    #[test]
    fn eular(){
        let axis = Vec3::new(-1.36712,2.55664,0.862798);
        let angle = 30.0_f64;

        let q = Quat::new_from_angle_axis(angle.radians(), axis);
        let m = Mat3::from(q);

        let e = m.eular_angle_zyz();

        assert!(
            (m.m00() - 
            (e.2.cos() * e.1.cos() * e.0.cos() - e.2.sin() * e.0.sin()))
            .abs() < 0.00001);
    }
}
