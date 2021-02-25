(function() {
    // Style
    let style = document.createElement("style");
    style.innerHTML = `
        #home > #log {
            height: 250px;
            resize: vertical;
            overflow-y: scroll;
        }
        #home > #log > pre {
            overflow-anchor: none;
            margin: 0px;
        };
        #home > #log > #anchor {
            overflow-anchor: auto;
            height: 1px;
        }
    `;
    document.head.appendChild(style);
    // Component
    app.component("component-home", {
        props: ["receive"], // Websocket hat Daten für component
        emits: ["send"], // component will Daten an Websocket weiterleiten
        watch: {
            receive(newVal, oldVal) {
                if (typeof newVal !== "string") {
                    // Infos erhalten
                    this.infos = newVal;
                } else {
                    // Log erhalten
                    this.logLines.push(newVal.replace(/\r?\n|\r/g, "").trim());
                }
            }
        },
        data() {
            return {
                counter: 0,
                logLines: [],
                infos: []
            }
        },
        template: `
            <div id="home">
                Info:<div id="info">
                    <pre v-for="info of infos">{{info}}</pre>
                </div>
                Log:<div id="log">
                    <pre v-for="line of logLines">{{line}}</pre>
                    <div id="anchor"></div>
                </div>
            </div>
        `,
        mounted() {
            let vm = this;
            setInterval(function() {
                vm.$emit("send", vm.counter++);
            }, 1000);
        }
    });
})();
