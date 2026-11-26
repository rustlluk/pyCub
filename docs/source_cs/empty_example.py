from icub_pybullet.pycub import pyCub

if __name__ == "__main__":
    # Načtení robota se zvolenou konfigurací světa
    client = pyCub(config="with_ball.yaml")

    # Smyčka simulace běží, dokud není okno/proces ukončen
    while client.is_alive():
        client.update_simulation()
