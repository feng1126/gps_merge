BUILD_DIR = ./build
DEST_DIR = ./bin
DEST_EXE_NAME = main		

all: clean build
	cd $(BUILD_DIR);  make -j4 ; cd ..; #mv -f ./bin/* ../
run:
	cd $(DEST_DIR); ./$(DEST_EXE_NAME)
clean:
	rm $(BUILD_DIR) -rf
build:
	mkdir $(BUILD_DIR);cd $(BUILD_DIR); cmake .. 

