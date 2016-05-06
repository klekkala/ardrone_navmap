all:
	@echo "making third party libs"
	cd libcvd && make
	cd gvars3 && make
	cd TooN && make
	cd dirent-1.20.1 && make
	cd eigen-3.2.1 && make
	cd openFabMap && mkdir -p build && cd build && cmake ../ && make


clean:
	@echo "cleaning up third party libs"
	cd libcvd && clean
	cd gvars3 && clean
	cd TooN && clean
	cd dirent-1.20.1 && clean
	cd eigen-3.2.1 && clean
	cd openFabMap && clean


eclipse-project:
	@echo "skip thirdparty."

