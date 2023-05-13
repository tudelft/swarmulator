#ifndef TYPES_H
#define TYPES_H

#include <iostream>
#include <iomanip>
#include <vector>
#include <functional>
#include <iterator> 
#include <cmath>
#include "utils.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include "math.h"

template<typename T>
class Vector{
  
  public:

		// constructors
		Vector(): _data(nullptr), _len(0.) {} //Default constructor
		Vector(int len):  _data(new T[len]), _len(len){
			// print(this);
		};
    	Vector(const std::initializer_list<T>& data): Vector((int)data.size()){
			std::uninitialized_copy(data.begin(), data.end(), _data);
		};

		// Copy constructor
		Vector(const Vector& other): Vector(other.get_len()){ 
			for (auto i=0; i<other.get_len(); i++){
				_data[i] = other[i];
			}
		}

		// Move constructor
		Vector(Vector&& other): _data(other.get_data()), _len(other.get_len()){ 
			other.set_len(0);
			other.set_data(nullptr);
		}

		// copy assignment
		Vector<T>& operator=(Vector<T>& other){
		if (this != &other){
			if (_len!=0) delete[] _data;
			_data = new T[other.get_len()];
			_len = other.get_len();
			for (auto i=0; i<_len; i++){
			_data[i] = other[i];
			}
		}
			return *this;
		}

		// move assignment
		Vector<T>& operator=(Vector<T>&& other){
			if (this!=&other){
				if (_len!=0) delete[] _data;
				_data = other.get_data();
				_len = other.get_len();
				other.set_data(nullptr);
				other.set_len(0);
			}
			return *this;
		}

		// destructor
		~Vector(){
			delete[] _data;
			_data  = nullptr;
			_len = 0;
			
		}

		// getters
		int get_len() const {return _len;}
		T* get_data() const {return _data;}
		T begin(){return _data[0];}
		T end(){return _data[0+_len];}

		// setters
		void set_len(int len){_len = len;}
		void set_data(T* data){_data = data;}

		// assignment operator[]
		T& operator[](const int i){
			return _data[i];
		}

		// access operator[]
		const T& operator[](const int i) const{ // read only const reference index 
			return _data[i];
		} 
		
		// scalar multiplication
		template <typename U>
	    Vector<typename std::common_type<T,U>::type> operator*(const U& other){
      Vector<typename std::common_type<T,U>::type> res(_len);
			for(int i = 0; i<_len;i++){
				res[i] = _data[i]*other;
			}
      return res;
    };

		// vector multiplication
		template <typename U>
		Vector<typename std::common_type<T,U>::type> operator*(const Vector<U>& other){
      Vector<typename std::common_type<T,U>::type> res(other.get_len());
			for(int i = 0; i<_len;i++){
				res[i] = _data[i]*other[i];
			}
      return res;
    };
		
		// vector addition
		template <typename U>
		Vector<typename std::common_type<T,U>::type> operator+(const Vector<U>& other){
      Vector<typename std::common_type<T,U>::type> res(other.get_len());
			for(int i = 0; i<_len;i++){
				res[i] = _data[i]+other[i];
			}
      return res;
    };

		// vector subtraction
		template <typename U>
		Vector<typename std::common_type<T,U>::type> operator-(const Vector<U>& other){
		Vector<typename std::common_type<T,U>::type> res(other.get_len());
				for(int i = 0; i<_len;i++){
					res[i] = _data[i]-other[i];
				}
		return res;
		};

		// in place addition
		template <typename U>
		Vector<T>& operator+=(const Vector<U>& other){
			for(int i = 0; i<_len; i++){
				_data[i] += other[i];
			}
      return *this;

	//   void append(T element){
		
	//   };
    };

		float mag(){
			float mag = 0;
			for(int i = 0; i<_len; i++){
				mag += pow(_data[i],2);
			}
			return sqrt(mag);
		}		

		// saturation
		Vector<T> saturate(float max = 6){
			if (mag()>max){
				normalise()*max;
			}
			return *this;
		}

		// normalisation
		Vector<T> normalise(){
			Vector<T> ret(_len);
			float mag = this->mag();
			if (abs(mag-0.0)<0.000001){
				return *this;
			}
			for(int i = 0; i<_len; i++){
				ret[i] = _data[i]/mag;
			}
			return ret;
		}	
		
		Eigen::Matrix<T, -1, 1> toEigen(){
			return Eigen::Map<Eigen::Matrix<T, -1, 1>>(_data, _len);
		};

		// help in printing
		friend std::ostream& operator<<(std::ostream& os, const Vector& vector){
			os << " [";
			for(int i = 0; i < vector.get_len(); i++) os << vector[i] << ", ";
			os << "] ";
    	return os;
  	}

	private:
    T* _data;
    int _len;

};

// template <typename T>
// std::ostream& operator<<(std::ostream& os, const T& state){
// 	std::cout << std::fixed;
// 	std::cout << std::setprecision(2);
// 	os << "Position: " << state.pos
// 		<< "Velociy: " << state.vel 
// 		<< "Accel: " << state.acc
// 		<< "Yaw: " << state.psi << " "
// 		<< "Ang. Vel.: " << state.omega;
// 	return os;
// }
template<typename T>
class Matrix{
	public:
		int _n_rows, _n_cols;
		Vector<Vector<T>> _mat;

	public:
		explicit Matrix(int n_rows, int n_cols): _n_rows(n_rows), _n_cols(n_cols){}

		~Matrix(){
			_n_rows = 0;
			_n_cols = 0;
		}

		// Assignment operator[]
		T& operator[](const std::pair<int, int>& ij){
			return _mat[ij];
		}

		// Access with exception
		const T& operator()(const std::pair<int, int>& ij) const {
			return _mat.at(ij);
		}

		// Vector product
		template<typename U>
		const Vector<typename std::common_type<T,U>::type> operator*(const Vector<U> vec){

		}

		// Matrix product
		template<typename U>
		const Matrix<typename std::common_type<T,U>::type> operator*(const Matrix<U> vec){
			Matrix<typename std::common_type<T,U>::type> prod;
			// for 
		}
};

class Pose{
	public:
		Pose():pos({0., 0., 0.}), quat({1., 0., 0., 0.,}){};
		Pose(Eigen::Vector3f _pos, Eigen::Vector4f _quat):pos(_pos), quat(_quat.normalized()){};
		
		// fake constructor sorta
		static Pose fromEuler(Eigen::Vector3f _pos, Eigen::Vector3f _rpy){
			// _pos = pos;

			float cr = cos(_rpy[0] * 0.5);
			float sr = sin(_rpy[0] * 0.5);
			float cp = cos(_rpy[1] * 0.5);
			float sp = sin(_rpy[1] * 0.5);
			float cy = cos(_rpy[2] * 0.5);
			float sy = sin(_rpy[2] * 0.5);

			Eigen::Vector4f _quat;
			_quat[0] = cr * cp * cy + sr * sp * sy;
			_quat[1] = sr * cp * cy - cr * sp * sy;
			_quat[2] = cr * sp * cy + sr * cp * sy;
			_quat[3] = cr * cp * sy - sr * sp * cy;

			return Pose(_pos, _quat);
			}

		Pose operator*(const Pose& other){
			Pose ret;
			ret.pos = pos + other.pos;

			// Quaternion multiplication

			// (a+bi+cj+dk)⋅(e+fi+gj+hk)
			Eigen::Vector4f _quat = other.quat;
			ret.quat[0] = quat[0]*_quat[0] - quat[1]*_quat[1] - quat[2]*_quat[2] - quat[3]*_quat[3];
			ret.quat[1] = quat[0]*_quat[1] + quat[1]*_quat[0] + quat[2]*_quat[3] - quat[3]*_quat[2];
			ret.quat[2] = quat[0]*_quat[2] - quat[1]*_quat[3] + quat[2]*_quat[0] + quat[3]*_quat[1];
			ret.quat[3] = quat[0]*_quat[3] + quat[1]*_quat[2] - quat[2]*_quat[1] + quat[3]*_quat[0];
 
			return ret;
		}
		
		

		// // Transform a vector using this pose
		// Vector<float> operator*(const Vector<float>& vec){

		// }

		Eigen::MatrixXf transform(Eigen::MatrixXf vecs){
			return (vecs * this->toRotMat().transpose()).rowwise() + pos.transpose();
		}

		Eigen::MatrixXf inv_transform(Eigen::MatrixXf vecs){
			return  (vecs.rowwise() - pos.transpose())* this->toRotMat();
		}
		

		Eigen::Vector4f toAxisAngle(){
			Eigen::Vector4f ret;

			if (quat[0] > 1) quat.normalize(); // if w>1 acos and sqrt will produce errors, this cant happen if quaternion is normalised
			ret[0] = 2 * acos(quat[0]);
			float s = sqrt(1-quat[0]*quat[0]); // assuming quaternion normalised then w is less than 1, so term always positive.
			if (s < 0.001) { // test to avoid divide by zero, s is always positive due to sqrt
				// if s close to zero then direction of axis not important
				ret[0] = quat[0]; // if it is important that axis is normalised then replace with x=1; y=z=0;
				ret[1] = quat[1];
				ret[2] = quat[2];
			} else {
				ret[0] = quat[0] / s; // normalise axis
				ret[1] = quat[1] / s;
				ret[2] = quat[2] / s;
			}
			return ret;
		}

		Eigen::MatrixXf toTransMat() {
			float q0 = quat[0];
			float q1 = quat[1];
			float q2 = quat[2];
			float q3 = quat[3];
			return Eigen::Matrix<float, 3, 4>{
				{2*(powf(q0,2) + powf(q1,2))-1 ,2*(q1*q2 - q0*q3)			  ,2*(q1*q3 + q0*q2)			 ,pos[0]},
				{2*(q1*q2 + q0*q3)			   ,2*(powf(q0,2) + powf(q2,2))-1 ,2*(q2*q3 - q0*q1)			 ,pos[1]},
				{2*(q1*q3 - q0*q2)			   ,2*(q2*q3 + q0*q1)			  ,2*(powf(q0,2) + powf(q3,2))-1 ,pos[2]},
			};
		}

		Eigen::Matrix3f toRotMat() {
			float q0 =quat[0];
			float q1 =quat[1];
			float q2 =quat[2];
			float q3 =quat[3];
			return Eigen::Matrix3f{
				{2*(powf(q0,2) + powf(q1,2))-1 ,2*(q1*q2 - q0*q3)			  ,2*(q1*q3 + q0*q2)			},
				{2*(q1*q2 + q0*q3)			   ,2*(powf(q0,2) + powf(q2,2))-1 ,2*(q2*q3 - q0*q1)			},
				{2*(q1*q3 - q0*q2)			   ,2*(q2*q3 + q0*q1)			  ,2*(powf(q0,2) + powf(q3,2))-1},
			};
		}

		Eigen::Vector3f toEuler() const {
			Eigen::Vector3f angles;
			float w = quat[0];
			float x = quat[1];
			float y = quat[2];
			float z = quat[3];
			// roll (x-axis rotation)
			double sinr_cosp = 2 * (w * x + y * z);
			double cosr_cosp = 1 - 2 * (x * x + y * y);
			angles[0] = std::atan2(sinr_cosp, cosr_cosp);

			// pitch (y-axis rotation)
			double sinp = std::sqrt(1 + 2 * (w * y - x * z));
			double cosp = std::sqrt(1 - 2 * (w * y - x * z));
			angles[1] = 2 * std::atan2(sinp, cosp) - M_PI / 2;

			// yaw (z-axis rotation)
			double siny_cosp = 2 * (w * z + x * y);
			double cosy_cosp = 1 - 2 * (y * y + z * z);
			angles[2] = std::atan2(siny_cosp, cosy_cosp);

			return angles;
		}
		// static Pose fromTransMat(Eigen::MatrixXf mat){
		// 	float q0 = sqrt(1.0 + ) / 2.0;
		// 	float w4 = (4.0 * q0);
		// 	float q1 = (mat(2,1) - mat(1,2)) / w4 ;
		// 	float q2 = (mat(0,2) - mat(2,0)) / w4 ;
		// 	float q3 = (mat(1,0) - mat(0,1)) / w4 ;
			
		// 	// method2
		// 	Eigen::EigenSolver<Eigen::MatrixXf> eigensolver(mat);
		// 	// eigensolver.compute(mat);
		// 	theta = std::acos(mat(0,0) + mat(1,1) + mat(2,2) - 1)/2);
		// 	e1 = eigensolver.eigenvectors().col(0);
			
		// 	q0 = std::cos(theta/2);
		// 	q1 = 

		// 	return Pose({mat(0,3), mat(1,3), mat(2,3)}, {q0,q1,q2,q3});

		// } 


	public:
		Eigen::Vector3f pos = {0., 0., 0.}; // x, y, z
		Eigen::Vector4f quat = {1., 0., 0., 0.}; // w, x, y, z


};


class State{
	public:
		// Vector<float> pos = {0.,0.,0.};
		Pose pose = Pose();
		Eigen::Vector3f vel = {0.,0.,0.};
		Eigen::Vector3f acc = {0.,0.,0.};
		float psi = 0.;  // yaw
		float omega = 0.; // yaw rate
		

    	// State() {};


		friend std::ostream& operator<<(std::ostream& os, const State& state){
			std::cout << std::fixed;
			std::cout << std::setprecision(2);
			os << "Position: " << state.pose.pos
			   << "Velociy: " << state.vel 
			   << "Accel: " << state.acc
				<< "Yaw: " << state.psi << " "
				<< "Ang. Vel.: " << state.omega;
			return os;
		}
};


#endif