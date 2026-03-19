#include "main_helper.h"
#include "My_Controller.h"

int main(int argc, char** argv) {
	main_helper(argc, argv, new MyController());

	return 0;
}
