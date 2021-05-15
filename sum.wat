(module 
    (type $sum_t (func (param i32) (param i32) (result i32)))
    (func $sum_f (type $sum_t) (param $a i32) (param $b i32) (result i32)
        local.get $a
        local.get $b
        i32.add
    )
    (export "sum" (func $sum_f))
)