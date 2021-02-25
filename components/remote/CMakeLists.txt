idf_component_register(SRCS "remote.c"
                      INCLUDE_DIRS "."
                      REQUIRES "esp_http_server" "mpack"
                      PRIV_REQUIRES "lwip" "esp_netif" "esp_event" "esp_wifi"
                      EMBED_FILES
                        "www/index.html"
                        "www/manifest.json"
                        "www/favicon.svg.gz"
                        "www/style.css"
                        "www/script.js"
                        "www/msgpack.min.js.gz"
                        "www/home.js"
                      )