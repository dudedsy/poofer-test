# Host simulator for the burst_capture firmware. `make` builds ./burst_sim.
FW = ../../firmware/burst_capture
SRCS = sim.cpp $(FW)/gfx.cpp $(FW)/screens.cpp $(FW)/ui.cpp $(FW)/store.cpp
burst_sim: $(SRCS) $(FW)/burst_capture.ino $(wildcard include/*.h include/hardware/*.h $(FW)/*.h)
	g++ -std=c++17 -O2 -g -Wall -Wno-unused-function -DBURST_SIM -Iinclude -I$(FW) -pthread \
	    -x c++ $(FW)/burst_capture.ino -x none $(SRCS) -o $@
clean:
	rm -f burst_sim
