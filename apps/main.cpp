#include <memory>

#include "main_helper.h"
#include "My_Controller.h"

int main(int argc, char** argv) {
	std::unique_ptr<MyController> myController = std::make_unique<MyController>();
	main_helper(argc, argv, myController.get());
	return 0;
}
