#![no_main]

extern "C"{
    fn foo(a :i32, b :i32)->i32;
    fn bar(a :i32, b :i32, c :i32, d :i32);
}

#[no_mangle]
pub extern fn ros_main() -> i32{
    unsafe{
        bar(1,2,3,4);
        foo(3,2)
    }
}