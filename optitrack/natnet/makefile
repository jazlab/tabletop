include=-Iinclude
libpath=-Llib
libs=-lNatNet

all:build/SampleClient

build/SampleClient:
	g++ src/SampleClient.cpp $(include) $(libpath) $(libs) -o build/SampleClient

.PHONY: clean
clean:
	@rm -f ./build/SampleClient
