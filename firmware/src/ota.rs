use defmt::{error, info};
use esp_bootloader_esp_idf::ota::OtaImageState;
use esp_bootloader_esp_idf::ota_updater::OtaUpdater;
use esp_bootloader_esp_idf::partitions::{
    self, AppPartitionSubType, PartitionTable, PartitionType,
};
use esp_storage::FlashStorage;

pub type Error = partitions::Error;

pub fn run_with_ota<F, R>(
    flash: &mut FlashStorage,
    partition_buf: &mut [u8; partitions::PARTITION_TABLE_MAX_LEN],
    operation: F,
) -> Result<R, Error>
where
    F: FnOnce(&mut OtaUpdater<FlashStorage>) -> R,
{
    let pt: PartitionTable = partitions::read_partition_table(flash, partition_buf)?;
    info!("Partition table len: {}", pt.len());

    let (ota0_offset, ota1_offset) = {
        let ota0_offset = pt
            .find_partition(PartitionType::App(AppPartitionSubType::Ota0))?
            .ok_or(Error::Invalid)?
            .offset();
        let ota1_offset = pt
            .find_partition(PartitionType::App(AppPartitionSubType::Ota1))?
            .ok_or(Error::Invalid)?
            .offset();
        (ota0_offset, ota1_offset)
    };
    info!("Ota0 offset {}, Ota1 offset {}", ota0_offset, ota1_offset);

    let mut ota = OtaUpdater::new(flash, partition_buf).map_err(|e| {
        error!("OTA init failed: {:?}", e);
        Error::Invalid
    })?;

    info!("OTA initialised successfully");

    Ok(operation(&mut ota))
}

#[allow(dead_code)]
pub fn set_next_ota_slot(ota: &mut OtaUpdater<FlashStorage>) -> Result<(), Error> {
    let (next_app_partition, _part_type) = ota.next_partition().unwrap();
    info!("Setting OTA slot to {:?}", next_app_partition);
    ota.activate_next_partition()?;
    ota.set_current_ota_state(OtaImageState::New)?;
    Ok(())
}

#[allow(dead_code)]
pub fn validate_current_ota_slot(ota: &mut OtaUpdater<FlashStorage>) -> Result<(), Error> {
    if let Ok(state) = ota.current_ota_state() {
        if state == OtaImageState::New || state == OtaImageState::PendingVerify {
            info!("Marking current OTA slot as VALID");
            ota.set_current_ota_state(OtaImageState::Valid).unwrap();
        }
    }

    Ok(())
}
