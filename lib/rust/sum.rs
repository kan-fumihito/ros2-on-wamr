#[no_mangle]
pub extern fn sum(a :i32, b :i32) -> i32{
    return a+b;
}

fn main(){
    sum(1,3);
}