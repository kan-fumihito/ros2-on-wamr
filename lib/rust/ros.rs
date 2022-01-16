#![no_main]

extern "C" {
    pub fn create_node_wasm(node_name: *const u8, len: u32) -> u32;
    pub fn create_publisher_wasm(node_id: u32, topic_name: *const u8, len: u32) -> u32;
    pub fn publish_wasm(publisher_id: u32, msg: *const u8, len: u32) -> ();
    pub fn spin_wasm(node_id: u32) -> ();
    pub fn wait_1000ms() -> ();
}

#[no_mangle]
pub extern fn ros_main(){
    unsafe{
        
        let mut node_id = create_node_wasm("node_wasm".as_ptr() as *const u8, 9);
        let mut publisher_id = create_publisher_wasm(node_id,"chatter".as_ptr() as *const u8, 7);
        
        for i in 0..100 {
            let msg = format!("Hello Rust ROS! {}", i);
            publish_wasm(publisher_id, msg.as_ptr() as *const u8, msg.chars().count() as u32);
            spin_wasm(node_id);
            wait_1000ms();
        
        }
    }
}