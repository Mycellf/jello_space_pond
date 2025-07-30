use macroquad::{
    input::{self, KeyCode},
    window::{self, Conf},
};

const START_IN_FULLSCREEN: bool = true;

fn config() -> Conf {
    Conf {
        window_title: "Jello Space Pond".to_owned(),
        fullscreen: START_IN_FULLSCREEN,
        ..Default::default()
    }
}

#[macroquad::main(config)]
async fn main() {
    let mut fullscreen = START_IN_FULLSCREEN;

    loop {
        if input::is_key_pressed(KeyCode::F11) {
            fullscreen ^= true;
            macroquad::window::set_fullscreen(fullscreen);
        }

        window::next_frame().await;
    }
}
