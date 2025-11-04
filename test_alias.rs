use serde::Deserialize;

#[derive(Debug, Deserialize)]
struct LayerData {
    #[serde(alias = "cognitive_layer_id")]
    dimension_id: u8,
    layer_num: u16,
}

fn main() {
    let json = r#"{"cognitive_layer_id": 1, "layer_num": 0}"#;
    let layer: LayerData = serde_json::from_str(json).unwrap();
    println!("Parsed: dimension_id={}, layer_num={}", layer.dimension_id, layer.layer_num);
}
