all: cvfpu softfloat ramulator cocogfx

cvfpu:

softfloat:
	SPECIALIZE_TYPE=RISCV SOFTFLOAT_OPTS="-fPIC -DSOFTFLOAT_ROUND_ODD -DINLINE_LEVEL=5 -DSOFTFLOAT_FAST_DIV32TO16 -DSOFTFLOAT_FAST_DIV64TO32 -DTHREAD_LOCAL=_Thread_local" $(MAKE) -C softfloat/build/Linux-x86_64-GCC

ramulator/libramulator.so:
	cd ramulator && mkdir -p build && cd build && cmake .. && make -j4

# The GDDR6 device model Ramulator ships but does not build. It registers with
# Ramulator's factory when the library loads, built as Ramulator builds itself.
RAMULATOR_EXT_DIR := ramulator_ext
RAMULATOR_EXT_INCS := -isystem ramulator/src -isystem ramulator/ext/spdlog/include -isystem ramulator/ext/yaml-cpp/include
RAMULATOR_EXT_CXXFLAGS := -std=c++20 -O3 -DNDEBUG -fPIC $(RAMULATOR_EXT_INCS)

# Order-only on the Ramulator build: its CMake step fetches the spdlog and
# yaml-cpp headers this object includes, so under -j it must finish first.
$(RAMULATOR_EXT_DIR)/gddr6.o: ramulator/src/dram/impl/GDDR6.cpp | ramulator/libramulator.so
	mkdir -p $(RAMULATOR_EXT_DIR)
	$(CXX) $(RAMULATOR_EXT_CXXFLAGS) -c $< -o $@

$(RAMULATOR_EXT_DIR)/libramulator-vortex.so: $(RAMULATOR_EXT_DIR)/gddr6.o ramulator/libramulator.so
	$(CXX) -shared $(filter %.o,$^) -Lramulator -lramulator -o $@

ramulator: ramulator/libramulator.so $(RAMULATOR_EXT_DIR)/libramulator-vortex.so

cocogfx/libcocogfx.a:
	$(MAKE) -C cocogfx
cocogfx: cocogfx/libcocogfx.a

clean:
	$(MAKE) -C softfloat/build/Linux-x86_64-GCC clean
	rm -rf ramulator/build ramulator/libramulator.so
	rm -f $(RAMULATOR_EXT_DIR)/*.o $(RAMULATOR_EXT_DIR)/*.so
	$(MAKE) -C cocogfx clean

.PHONY: all cvfpu softfloat ramulator cocogfx