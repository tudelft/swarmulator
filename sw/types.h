#ifndef TYPES_H
#define TYPES_H
#include <iostream>
#include <iomanip>
#include <vector>
#include <functional>
#include <iterator> 
#include "utils.h"

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

class State{
	public:
		Vector<float> pos = {0.,0.,0.};
		Vector<float> vel = {0.,0.,0.};
		Vector<float> acc = {0.,0.,0.};
		float psi = 0.;  // yaw
		float omega = 0.; // yaw rate

    	// State() {};


		friend std::ostream& operator<<(std::ostream& os, const State& state){
			std::cout << std::fixed;
			std::cout << std::setprecision(2);
			os << "Position: " << state.pos
			   << "Velociy: " << state.vel 
			   << "Accel: " << state.acc
				<< "Yaw: " << state.psi << " "
				<< "Ang. Vel.: " << state.omega;
			return os;
		}
};

class Pose{
	public:
		Pose():_pos({0., 0., 0.}), _quat({0., 0., 0., 0.,}){};
		Pose(Vector<float>pos, Vector<float> quat):_pos(pos), _quat(quat){};

		Pose operator*(const Pose& other){
			Pose ret;
			ret.setPos(_pos + other.getPos());

			// Quaternion multiplication

			// (a+bi+cj+dk)⋅(e+fi+gj+hk)
			Vector<float> quat = other.getQuat();
			ret.setQuat(0, _quat[0]*quat[0] - _quat[1]*quat[1] - _quat[2]*quat[2] - _quat[3]*quat[3]);
			ret.setQuat(1, _quat[0]*quat[1] + _quat[1]*quat[0] + _quat[2]*quat[3] - _quat[3]*quat[2]);
			ret.setQuat(2, _quat[0]*quat[2] - _quat[1]*quat[3] + _quat[2]*quat[0] + _quat[3]*quat[1]);
			ret.setQuat(3, _quat[0]*quat[3] + _quat[1]*quat[2] - _quat[2]*quat[1] + _quat[3]*quat[0]);
 
			return ret;
		}

		Vector<float> getPos() const {return _pos;}
		void setPos(Vector<float> pos){_pos = pos;}
		
		Vector<float> getQuat() const {return _quat;}
		Vector<float> getQuat(const int dim) const {return _quat[dim];}
		void setQuat(Vector<float> quat){_quat = quat;}
		void setQuat(int dim, float element){_quat[dim] = element;}

	private:
		Vector<float> _pos = {0., 0., 0.}; // x, y, z
		Vector<float> _quat = {1., 0., 0., 0.}; // w, x, y, z




};

#endif