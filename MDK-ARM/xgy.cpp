#include "Matrix.h"
Matrix<2,2> A_test;
Matrix<2,2> B_test;
Matrix<2,2> C_test;
Matrix<2,2> D_test;
arm_status ret;
extern "C" void	test(){
	
	A_test.data[0][0] = 1;A_test.data[0][1] = 0;
	A_test.data[1][0] = 0;A_test.data[1][1] = 5;
	
	
	
	B_test(0,0) = 2;B_test(0,1) = 0;
	B_test(1,0) = 0;B_test(1,1) = 3;
	
	
	C_test = A_test * B_test;
	
	C_test = B_test * A_test;
	
	C_test = A_test * 2;
	
	C_test = 2 * A_test;
	
			A_test.data[0][0] = 1;A_test.data[0][1] = 0;
	A_test.data[1][0] = 0;A_test.data[1][1] = 5;
	
	C_test = A_test.transpose();
	
	ret = A_test.inv(D_test);
	

	
	
	
	B_test(0,0) = 2;B_test(0,1) = 0;
	B_test(1,0) = 0;B_test(1,1) = 3;
	
	
//	C_test = A_test * B_test;
//	
//	C_test = B_test * A_test;
	
		B_test(0,0) = 2;B_test(0,1) = 0;
	B_test(1,0) = 0;B_test(1,1) = 3;
	
	
	A_test = A_test * B_test;
	
	A_test = B_test * A_test;
}




