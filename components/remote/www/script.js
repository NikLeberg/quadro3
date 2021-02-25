// Funktionen

// Vue App
const app = Vue.createApp({
    data() {
        return {
            currentTab: "",
            tabs: [],
            ws: null,
            rxData: [],
            txBacklog: []
        }
    },
    methods: {
        wsInit() {
            let vm = this;
            vm.ws = new WebSocket("ws:/quadro3.local/ws");
            vm.ws.binaryType = "arraybuffer";
            vm.ws.onmessage = (event) => {
                vm.wsReceive(event.data);
            };
            vm.ws.onopen = this.wsSendBacklog;
        },
        wsSend(handle, data) {
            let message = msgpack.serialize([handle, data]);
            if (this.ws && this.ws.readyState === 1) {
                this.ws.send(message);
            } else {
                this.txBacklog.push(message);
            }
        },
        wsSendBacklog() {
            for (message of this.txBacklog) {
                this.ws.send(message);
            }
            this.txBacklog = [];
        },
        wsReceive(arrayBuffer) {
            let deserialized = msgpack.deserialize(arrayBuffer);
            let handle = deserialized[0] - 1;
            let data = deserialized[1];
            this.rxData[handle] = data;
        },
        loadTabs() {
            let vm = this;
            let ajax = new XMLHttpRequest();
            ajax.onload = () => {
                let components = msgpack.deserialize(ajax.response);
                for (let [i, name] of components.entries()) {
                    if (name) this.addTab(i + 1, name);
                }
            }
            ajax.open("GET", "/component");
            ajax.responseType = "arraybuffer";
            ajax.send();
        },
        addTab(handle, name) {
            let vm = this;
            if (name) {
                let script = document.createElement("script");
                document.head.appendChild(script);
                script.onload = () => {
                    vm.tabs.unshift({handle: handle, name: name});
                    if (!vm.currentTab) {
                        vm.currentTab = name;
                    }
                };
                script.src = "/component?" + name;
            }
        }
    },
    computed: {
    },
    mounted() {
        this.loadTabs();
        let vm = this;
        setTimeout(() => {
            vm.wsInit(); // ToDo: erst Starten wenn alle Components geladen
        }, 2000);
    }
});

var vm = app.mount("#app");
