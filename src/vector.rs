use std::fmt;
use std::f64::consts::PI;
use std::ops::{Add, Mul, Sub};
use std::convert::From;

use super::quaternion::Quat;
use super::matrix3::Mat3;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vec3 (f64, f64, f64);

impl Add for Vec3 {
	type Output = Vec3;

	fn add (self, other:Vec3) -> Vec3 {
		// this is the cross product
		Vec3(
			self.0 + other.0,
			self.1 + other.1,
			self.2 + other.2
		)
	}
}

impl Mul for Vec3 {
	type Output = Vec3;

	fn mul (self, other:Vec3) -> Vec3 {
		// this is the cross product
		Vec3(
			self.1 * other.2 - self.2 * other.1,
			self.2 * other.0 - self.0 * other.2,
			self.0 * other.1 - self.1 * other.0
		)
	}
}

impl Sub for Vec3 {
	type Output = Vec3;

	fn sub (self, other: Vec3) -> Vec3 {
		// this is the cross product
		Vec3(
			self.0 - other.0,
			self.1 - other.1,
			self.2 - other.2
		)
	}
}
impl From<Quat> for Vec3 {
	fn from(original: Quat) -> Vec3 {
		Vec3(original.x(), original.y(), original.z())
	}
}

impl Vec3 {
	pub fn new(x:f64, y:f64, z:f64) -> Vec3{Vec3(x, y, z)}	

	#[allow(dead_code)]
	pub fn zero() -> Vec3{Vec3(0.0, 0.0, 0.0)}	

	#[allow(dead_code)]
	pub fn unit_x() -> Vec3{Vec3(1.0, 0.0, 0.0)}	

	#[allow(dead_code)]
	pub fn unit_z() -> Vec3{Vec3(0.0, 0.0, 1.0)}	

	pub fn normal_from_three_vertices(a:Vec3, b: Vec3, c:Vec3) -> Vec3{
		let ab = b - a;
		let ac = c - a;
		(ab * ac).unitize()
	}

	// FIXME: not generic
	pub fn normal_from_vertex_array(v:[Vec3;3]) -> Vec3{
		Vec3::normal_from_three_vertices(v[0], v[1], v[2])
	}

	pub fn mean_from_three_vertices(a:Vec3, b: Vec3, c:Vec3) -> Vec3{
		let sum = a + b + c;
		sum.scalar_div(3.0)
	}

	// FIXME: not generic
	pub fn mean_from_vertex_array(v:[Vec3;3]) -> Vec3{
		Vec3::mean_from_three_vertices(v[0], v[1], v[2])
	}

	pub fn x(&self) -> f64 {
		self.0
	}

	pub fn y(&self) -> f64 {
		self.1
	}

	pub fn z(&self) -> f64 {
		self.2
	}

	pub fn apply_rot_mat3(&self, m:Mat3) -> Vec3 {
		Vec3(
			self.0 * m.m00() + self.1 * m.m01() + self.2 * m.m02(),
			self.0 * m.m10() + self.1 * m.m11() + self.2 * m.m12(),
			self.0 * m.m20() + self.1 * m.m21() + self.2 * m.m22(),
		)
	}

	pub fn distance(self, other:Vec3) -> f64 {
		(self - other).length()
	}


	pub fn dot(&self, other: &Vec3) -> f64 {
		self.0 * other.0 + self.1 * other.1 + self.2 * other.2
	}

	// FIXME: can I override ops::Mul??
	pub fn scalar_mul(&self, s:f64) -> Vec3{
		Vec3(
			self.0 * s,
			self.1 * s,
			self.2 * s
		)
	}

	pub fn scalar_div(&self, s:f64) -> Vec3{
		Vec3(
			self.0 / s,
			self.1 / s,
			self.2 / s
		)
	}

	pub fn length (&self) -> f64 {
		(self.0 * self.0 + self.1 * self.1 + self.2 *self.2).sqrt()
	}

	// normalize??
	pub fn unitize(&self) -> Vec3 {
		let length = self.length(); 
		Vec3(self.0/length, self.1/length, self.2/length)	
	}

	pub fn angle(&self, other: &Vec3) -> f64 {
		let d = self.dot(other);
		let lengths  = self.length() * other.length();
		(d/lengths).acos()		
	}
}

pub trait Angle {		
	fn degrees(&self) -> f64;
	fn radians(&self) -> f64;	
}

impl Angle for f64{
	fn degrees(&self) -> f64 {
		self / PI * 180.0
	}

	fn radians(&self) -> f64 {
		self * PI / 180.0
	}
} 

impl fmt::Display for Vec3 {
	fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
		write!(f, "({}, {}, {})", self.0, self.1, self.2)
	}
}
