fn main() {
    tonic_build::configure()
        .build_server(true)
        .compile(
            &["farm-ng-amiga/protos/farm_ng/canbus/canbus.proto"],
            &[
                "farm-ng-amiga/protos/",
                "farm-ng-amiga/protos/farm_ng/canbus/",
            ],
        )
        .unwrap();
}
