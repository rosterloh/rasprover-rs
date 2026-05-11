ESPFLASH_ARGS = --chip esp32 \
              --partition-table=./partitions.csv \
              -s 4mb \
              target/xtensa-esp32-none-elf/release/rasprover

deps:
	echo "Installing dependencies"
	cargo install espup
	rustup default esp
	espup install
	. $HOME/export-esp.sh

clean:
	cargo clean
	rm -rf output/firmware.bin

build:
    . .env
	cargo build

lint:
	cargo clippy --workspace --release

release: clean
    . .env
	cargo build --release

stats:
	xtensa-esp32-elf-size -A target/xtensa-esp32-none-elf/release/rasprover

dram-usage:
	cargo bloat --release --crates
	cargo bloat --release --functions

firmware:
	mkdir -p output
	espflash save-image ${ESPFLASH_ARGS} output/firmware.bin
	espflash partition-table --to-binary partitions.csv > output/partitions.bin
	cp -r partitions.csv output/partitions.csv
	chmod 777 output/partitions.csv
	chmod 777 output/firmware.bin
	chmod 777 output/partitions.bin

erase:
	espflash erase-flash

flash:
	espflash flash ${ESPFLASH_ARGS} -B 921600

flash-firmware:
	espflash write-bin --chip esp32 0x8000 output/partitions.bin
	espflash write-bin --chip esp32 0x10000 output/firmware.bin

monitor:
	espflash monitor

run:
    . .env
	cargo run