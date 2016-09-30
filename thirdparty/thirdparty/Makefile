all:
	@echo "making third party libs"
	cd libcvd && make
	cd gvars3 && make
	cd TooN && make
	cd DBoW2/build && make
	cd g2o/build && make


clean:
	@echo "cleaning up third party libs"
	cd libcvd && make clean  
	cd gvars3 && make clean
	cd DBoW2/build && make clean
	cd g2o/build && make clean

eclipse-project:
	@echo "skip thirdparty."

