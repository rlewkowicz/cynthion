#[doc = r"Register block"]
#[repr(C)]
pub struct RegisterBlock {
    #[doc = "0x00 - gpioa moder register"]
    pub moder: MODER,
    #[doc = "0x04 - gpioa odr register"]
    pub odr: ODR,
    #[doc = "0x08 - gpioa idr register"]
    pub idr: IDR,
    _reserved3: [u8; 0x04],
    #[doc = "0x10 - gpioa ev_status register"]
    pub ev_status: EV_STATUS,
    #[doc = "0x14 - gpioa ev_pending register"]
    pub ev_pending: EV_PENDING,
    #[doc = "0x18 - gpioa ev_enable register"]
    pub ev_enable: EV_ENABLE,
}
#[doc = "moder (rw) register accessor: an alias for `Reg<MODER_SPEC>`"]
pub type MODER = crate::Reg<moder::MODER_SPEC>;
#[doc = "gpioa moder register"]
pub mod moder;
#[doc = "odr (w) register accessor: an alias for `Reg<ODR_SPEC>`"]
pub type ODR = crate::Reg<odr::ODR_SPEC>;
#[doc = "gpioa odr register"]
pub mod odr;
#[doc = "idr (r) register accessor: an alias for `Reg<IDR_SPEC>`"]
pub type IDR = crate::Reg<idr::IDR_SPEC>;
#[doc = "gpioa idr register"]
pub mod idr;
#[doc = "ev_status (r) register accessor: an alias for `Reg<EV_STATUS_SPEC>`"]
pub type EV_STATUS = crate::Reg<ev_status::EV_STATUS_SPEC>;
#[doc = "gpioa ev_status register"]
pub mod ev_status;
#[doc = "ev_pending (rw) register accessor: an alias for `Reg<EV_PENDING_SPEC>`"]
pub type EV_PENDING = crate::Reg<ev_pending::EV_PENDING_SPEC>;
#[doc = "gpioa ev_pending register"]
pub mod ev_pending;
#[doc = "ev_enable (rw) register accessor: an alias for `Reg<EV_ENABLE_SPEC>`"]
pub type EV_ENABLE = crate::Reg<ev_enable::EV_ENABLE_SPEC>;
#[doc = "gpioa ev_enable register"]
pub mod ev_enable;
