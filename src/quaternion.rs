use super::vector::{Angle, Vec3};
use std::convert::From;
use std::ops::{Add, Mul};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Quat(f64, f64, f64, f64);

impl From<Vec3> for Quat {
  fn from(original: Vec3) -> Quat {
    Quat(0.0, original.x(), original.y(), original.z())
  }
}

impl Add for Quat {
  type Output = Quat;
  fn add(self, other: Quat) -> Quat {
    Quat(
      self.0 + other.w(),
      self.1 + other.x(),
      self.2 + other.y(),
      self.3 + other.z(),
    )
  }
}

impl Mul for Quat {
  type Output = Quat;

  fn mul(self, other: Quat) -> Quat {
    Quat(
      self.w() * other.w() - self.x() * other.x() - self.y() * other.y() - self.z() * other.z(),
      self.w() * other.x() + self.x() * other.w() + self.y() * other.z() - self.z() * other.y(),
      self.w() * other.y() + self.y() * other.w() + self.z() * other.x() - self.x() * other.z(),
      self.w() * other.z() + self.z() * other.w() + self.x() * other.y() - self.y() * other.x(),
    )
  }
}

impl Quat {
  #[allow(dead_code)]
  pub fn zero() -> Quat {
    Quat(0.0, 0.0, 0.0, 0.0)
  }

  pub fn new(w: f64, x: f64, y: f64, z: f64) -> Quat {
    Quat(w, x, y, z)
  }

  pub fn new_from_angle_axis(rad: f64, axis: Vec3) -> Quat {
    let axis = axis.unitize();
    let sin_halve_theta = (rad * 0.5).sin();

    Quat(
      (rad * 0.5).cos(),
      axis.x() * sin_halve_theta,
      axis.y() * sin_halve_theta,
      axis.z() * sin_halve_theta,
    )
  }

  pub fn new_from_vec(w: f64, v: Vec3) -> Quat {
    Quat::new(w, v.x(), v.y(), v.z())
  }

  pub fn w(&self) -> f64 {
    self.0
  }
  pub fn x(&self) -> f64 {
    self.1
  }
  pub fn y(&self) -> f64 {
    self.2
  }
  pub fn z(&self) -> f64 {
    self.3
  }

  pub fn length(&self) -> f64 {
    (self.0.powi(2) + self.1.powi(2) + self.2.powi(2) + self.3.powi(2)).sqrt()
  }

  pub fn scalar_mul(&self, s: f64) -> Quat {
    Quat(self.0 * s, self.1 * s, self.2 * s, self.3 * s)
  }

  pub fn unitize(&self) -> Quat {
    let length = self.length();
    Quat(
      self.0 / length,
      self.1 / length,
      self.2 / length,
      self.3 / length,
    )
  }

  pub fn conjugate(&self) -> Quat {
    // let v = Vec3::from(self).scalar_mul(-1.0);
    Quat(self.0, -self.1, -self.2, -self.3)
  }

  pub fn rotate_vec3(self, v: Vec3) -> Vec3 {
    Vec3::from(self * Quat::from(v) * self.conjugate())
  }

  pub fn angle_axis(self) -> (f64, Vec3) {
    let u = self.unitize();

    let angle = u.w().acos() * 2.0;

    let sin_halve_theta = (angle * 0.5).sin();

    let v = Vec3::from(self).scalar_div(sin_halve_theta);

    (angle, v)
  }

  pub fn rot_between_vecs(start: Vec3, dest: Vec3) -> Quat {
    let start = start.unitize();
    let dest = dest.unitize();

    let cos_theta = start.dot(&dest);

    if cos_theta < -1.0 + 0.001 {
      let mut rot_axis = Vec3::unit_z() * start;
      if rot_axis.length() < 0.01 {
        rot_axis = Vec3::unit_x() * start;
      }
      rot_axis = rot_axis.unitize();

      // make quat from angle and axis
      Quat::new_from_angle_axis(180.0_f64.radians(), rot_axis)
    } else {

      let rot_axis = start * dest;
      let s = ((1.0 + cos_theta) * 2.0).sqrt();
      let invs = 1.0 / s;

      Quat(
        s * 0.5,
        rot_axis.x()*invs,
        rot_axis.y()*invs,
        rot_axis.z()*invs)
    }
  }
}
