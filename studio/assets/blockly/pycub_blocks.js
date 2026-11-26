/* Offline Blockly custom robot blocks and Python code generators for pyCub Scratch Mode */
(function () {
  var CURRENT_LANG = "en";

  var LABELS = {
    en: {
      cat_presets: "🤖 Robot Actions",
      cat_arms: "💪 Arms",
      cat_head_body: "👀 Head & Body",
      cat_sensors: "⚽ Sensors",
      cat_loops: "🔁 Loops",
      cat_logic: "🧠 Logic",
      cat_math: "🔢 Math & Text",
      cat_vars: "📦 Variables",
      right: "right",
      left: "left",
      slow: "🐢 slow",
      medium: "🚶 medium",
      fast: "🐇 fast",
      wave: "👋 wave with %1 arm",
      both_arms: "🙌 move both arms %1",
      up: "up",
      down: "down",
      look_center: "🎯 look straight ahead",
      stand_straight: "🧍 stand straight",
      reset_pose: "🔄 reset robot pose",
      wait: "⏱️ wait %1 seconds",
      arm_move: "💪 %1 arm: %2 by %3 ° (%4)",
      shoulder_up: "shoulder up",
      shoulder_down: "shoulder down",
      shoulder_out: "shoulder out",
      shoulder_in: "shoulder in",
      rotate_arm: "rotate arm",
      bend_elbow: "bend elbow",
      straighten_elbow: "straighten elbow",
      rotate_wrist: "rotate wrist",
      head_move: "👀 head: %1 by %2 ° (%3)",
      look_up: "look up",
      look_down: "look down",
      turn_head: "turn (+left / -right)",
      tilt_head: "tilt (+left / -right)",
      body_move: "🧍 body: %1 by %2 ° (%3)",
      turn_body: "turn (+left / -right)",
      bend_forward: "bend forward",
      bend_back: "bend back",
      bend_sideways: "bend sideways",
      ball_touched: "⚽ ball touched?",
      skin_touched: "🖐️ tactile skin touched?",
      ball_distance: "📏 ball distance from table (cm)",
      ball_hits: "🏆 ball hit count",
      repeat_forever: "🔁 repeat forever %1 %2"
    },
    cs: {
      cat_presets: "🤖 Hotové pohyby",
      cat_arms: "💪 Ruce",
      cat_head_body: "👀 Hlava a tělo",
      cat_sensors: "⚽ Senzory",
      cat_loops: "🔁 Cykly",
      cat_logic: "🧠 Podmínky",
      cat_math: "🔢 Čísla a výpis",
      cat_vars: "📦 Proměnné",
      right: "pravou",
      left: "levou",
      slow: "🐢 pomalu",
      medium: "🚶 středně",
      fast: "🐇 rychle",
      wave: "👋 zamávat %1 rukou",
      both_arms: "🙌 obě ruce %1",
      up: "nahoru",
      down: "dolů",
      look_center: "🎯 dívat se rovně",
      stand_straight: "🧍 narovnat tělo",
      reset_pose: "🔄 základní postoj",
      wait: "⏱️ počkat %1 s",
      arm_move: "💪 %1 ruka: %2 o %3 ° (%4)",
      shoulder_up: "rameno nahoru",
      shoulder_down: "rameno dolů",
      shoulder_out: "rameno do strany",
      shoulder_in: "rameno k tělu",
      rotate_arm: "otočit paži",
      bend_elbow: "ohnout loket",
      straighten_elbow: "natáhnout loket",
      rotate_wrist: "otočit zápěstí",
      head_move: "👀 hlava: %1 o %2 ° (%3)",
      look_up: "podívat nahoru",
      look_down: "podívat dolů",
      turn_head: "otočit (+vlevo / -vpravo)",
      tilt_head: "naklonit (+vlevo / -vpravo)",
      body_move: "🧍 tělo: %1 o %2 ° (%3)",
      turn_body: "otočit (+vlevo / -vpravo)",
      bend_forward: "předklonit",
      bend_back: "zaklonit",
      bend_sideways: "uklonit do strany",
      ball_touched: "⚽ dotkl se míčku?",
      skin_touched: "🖐️ dotyk na kůži?",
      ball_distance: "📏 vzdálenost míčku od stolu (cm)",
      ball_hits: "🏆 počet zásahů míčku",
      repeat_forever: "🔁 opakuj pořád %1 %2"
    }
  };

  function L(key) {
    var table = LABELS[CURRENT_LANG] || LABELS.en;
    return table[key] || LABELS.en[key] || key;
  }

  function registerBlocks() {
    var sideOpts = function () {
      return [[L("right"), "right"], [L("left"), "left"]];
    };
    var speedOpts = function () {
      return [[L("slow"), "0.6"], [L("medium"), "1.5"], [L("fast"), "2.6"]];
    };

    Blockly.Blocks["pycub_wave"] = {
      init: function () {
        this.jsonInit({
          message0: L("wave"),
          args0: [{ type: "field_dropdown", name: "SIDE", options: sideOpts() }],
          previousStatement: null,
          nextStatement: null,
          colour: "#2563eb"
        });
      }
    };

    Blockly.Blocks["pycub_both_arms"] = {
      init: function () {
        this.jsonInit({
          message0: L("both_arms"),
          args0: [{
            type: "field_dropdown",
            name: "DIR",
            options: [[L("up"), "up"], [L("down"), "down"]]
          }],
          previousStatement: null,
          nextStatement: null,
          colour: "#2563eb"
        });
      }
    };

    Blockly.Blocks["pycub_look_center"] = {
      init: function () {
        this.jsonInit({
          message0: L("look_center"),
          previousStatement: null,
          nextStatement: null,
          colour: "#2563eb"
        });
      }
    };

    Blockly.Blocks["pycub_stand_straight"] = {
      init: function () {
        this.jsonInit({
          message0: L("stand_straight"),
          previousStatement: null,
          nextStatement: null,
          colour: "#2563eb"
        });
      }
    };

    Blockly.Blocks["pycub_reset_pose"] = {
      init: function () {
        this.jsonInit({
          message0: L("reset_pose"),
          previousStatement: null,
          nextStatement: null,
          colour: "#2563eb"
        });
      }
    };

    Blockly.Blocks["pycub_wait"] = {
      init: function () {
        this.jsonInit({
          message0: L("wait"),
          args0: [{ type: "field_number", name: "SEC", value: 1, min: 0, max: 60, precision: 0.1 }],
          previousStatement: null,
          nextStatement: null,
          colour: "#0284c7"
        });
      }
    };

    Blockly.Blocks["pycub_arm_move"] = {
      init: function () {
        this.jsonInit({
          message0: L("arm_move"),
          args0: [
            { type: "field_dropdown", name: "SIDE", options: sideOpts() },
            {
              type: "field_dropdown",
              name: "ACT",
              options: [
                [L("shoulder_up"), "shoulder_up"],
                [L("shoulder_down"), "shoulder_down"],
                [L("shoulder_out"), "shoulder_out"],
                [L("shoulder_in"), "shoulder_in"],
                [L("rotate_arm"), "rotate_arm"],
                [L("bend_elbow"), "bend_elbow"],
                [L("straighten_elbow"), "straighten_elbow"],
                [L("rotate_wrist"), "rotate_wrist"]
              ]
            },
            { type: "field_number", name: "DEG", value: 15, min: -90, max: 90, precision: 1 },
            { type: "field_dropdown", name: "SPD", options: speedOpts() }
          ],
          previousStatement: null,
          nextStatement: null,
          colour: "#7c3aed"
        });
      }
    };

    Blockly.Blocks["pycub_head_move"] = {
      init: function () {
        this.jsonInit({
          message0: L("head_move"),
          args0: [
            {
              type: "field_dropdown",
              name: "ACT",
              options: [
                [L("look_up"), "look_up"],
                [L("look_down"), "look_down"],
                [L("turn_head"), "turn_head"],
                [L("tilt_head"), "tilt_head"]
              ]
            },
            { type: "field_number", name: "DEG", value: 15, min: -60, max: 60, precision: 1 },
            { type: "field_dropdown", name: "SPD", options: speedOpts() }
          ],
          previousStatement: null,
          nextStatement: null,
          colour: "#0891b2"
        });
      }
    };

    Blockly.Blocks["pycub_body_move"] = {
      init: function () {
        this.jsonInit({
          message0: L("body_move"),
          args0: [
            {
              type: "field_dropdown",
              name: "ACT",
              options: [
                [L("bend_forward"), "bend_forward"],
                [L("bend_back"), "bend_back"],
                [L("turn_body"), "turn_body"],
                [L("bend_sideways"), "bend_sideways"]
              ]
            },
            { type: "field_number", name: "DEG", value: 15, min: -60, max: 60, precision: 1 },
            { type: "field_dropdown", name: "SPD", options: speedOpts() }
          ],
          previousStatement: null,
          nextStatement: null,
          colour: "#0891b2"
        });
      }
    };

    Blockly.Blocks["pycub_ball_touched"] = {
      init: function () {
        this.jsonInit({
          message0: L("ball_touched"),
          output: "Boolean",
          colour: "#d97706"
        });
      }
    };

    Blockly.Blocks["pycub_skin_touched"] = {
      init: function () {
        this.jsonInit({
          message0: L("skin_touched"),
          output: "Boolean",
          colour: "#d97706"
        });
      }
    };

    Blockly.Blocks["pycub_ball_distance"] = {
      init: function () {
        this.jsonInit({
          message0: L("ball_distance"),
          output: "Number",
          colour: "#d97706"
        });
      }
    };

    Blockly.Blocks["pycub_ball_hits"] = {
      init: function () {
        this.jsonInit({
          message0: L("ball_hits"),
          output: "Number",
          colour: "#d97706"
        });
      }
    };

    Blockly.Blocks["pycub_forever"] = {
      init: function () {
        this.jsonInit({
          message0: L("repeat_forever"),
          args0: [
            { type: "input_dummy" },
            { type: "input_statement", name: "DO" }
          ],
          previousStatement: null,
          colour: "#16a34a"
        });
      }
    };
  }

  function registerGenerators() {
    var py = (window.python && window.python.pythonGenerator) ? window.python.pythonGenerator : Blockly.Python;
    var Order = (window.python && window.python.Order) ? window.python.Order : { FUNCTION_CALL: 2 };
    var reg = py.forBlock || py;

    reg["pycub_wave"] = function (block) {
      return 'wave("' + block.getFieldValue("SIDE") + '")\n';
    };
    reg["pycub_both_arms"] = function (block) {
      return block.getFieldValue("DIR") === "up" ? "both_arms_up()\n" : "both_arms_down()\n";
    };
    reg["pycub_look_center"] = function () {
      return "look_center()\n";
    };
    reg["pycub_stand_straight"] = function () {
      return "stand_straight()\n";
    };
    reg["pycub_reset_pose"] = function () {
      return "reset_pose()\n";
    };
    reg["pycub_wait"] = function (block) {
      return "wait(" + Number(block.getFieldValue("SEC")) + ")\n";
    };
    reg["pycub_arm_move"] = function (block) {
      var side = block.getFieldValue("SIDE");
      var act = block.getFieldValue("ACT");
      var deg = Number(block.getFieldValue("DEG"));
      var spd = Number(block.getFieldValue("SPD"));
      return act + '("' + side + '", ' + deg + ", speed=" + spd + ")\n";
    };
    reg["pycub_head_move"] = function (block) {
      var act = block.getFieldValue("ACT");
      var deg = Number(block.getFieldValue("DEG"));
      var spd = Number(block.getFieldValue("SPD"));
      return act + "(" + deg + ", speed=" + spd + ")\n";
    };
    reg["pycub_body_move"] = function (block) {
      var act = block.getFieldValue("ACT");
      var deg = Number(block.getFieldValue("DEG"));
      var spd = Number(block.getFieldValue("SPD"));
      return act + "(" + deg + ", speed=" + spd + ")\n";
    };
    reg["pycub_ball_touched"] = function () {
      return ["ball_touched()", Order.FUNCTION_CALL];
    };
    reg["pycub_skin_touched"] = function () {
      return ["skin_touched()", Order.FUNCTION_CALL];
    };
    reg["pycub_ball_distance"] = function () {
      return ["ball_distance()", Order.FUNCTION_CALL];
    };
    reg["pycub_ball_hits"] = function () {
      return ["ball_hits()", Order.FUNCTION_CALL];
    };
    reg["pycub_forever"] = function (block, generator) {
      var gen = generator || py;
      var branch = gen.statementToCode(block, "DO");
      if (!branch || !branch.trim()) {
        branch = gen.INDENT + "wait(0.1)\n";
      }
      return "while True:\n" + branch;
    };
  }

  function buildToolbox() {
    return {
      kind: "categoryToolbox",
      contents: [
        {
          kind: "category",
          name: L("cat_presets"),
          colour: "#2563eb",
          contents: [
            { kind: "block", type: "pycub_wave" },
            { kind: "block", type: "pycub_both_arms" },
            { kind: "block", type: "pycub_look_center" },
            { kind: "block", type: "pycub_stand_straight" },
            { kind: "block", type: "pycub_reset_pose" },
            { kind: "block", type: "pycub_wait" }
          ]
        },
        {
          kind: "category",
          name: L("cat_arms"),
          colour: "#7c3aed",
          contents: [
            { kind: "block", type: "pycub_arm_move", fields: { SIDE: "right", ACT: "shoulder_up", DEG: 15, SPD: "1.5" } },
            { kind: "block", type: "pycub_arm_move", fields: { SIDE: "right", ACT: "bend_elbow", DEG: 20, SPD: "1.5" } },
            { kind: "block", type: "pycub_arm_move", fields: { SIDE: "left", ACT: "shoulder_up", DEG: 15, SPD: "1.5" } },
            { kind: "block", type: "pycub_arm_move", fields: { SIDE: "left", ACT: "bend_elbow", DEG: 20, SPD: "1.5" } }
          ]
        },
        {
          kind: "category",
          name: L("cat_head_body"),
          colour: "#0891b2",
          contents: [
            { kind: "block", type: "pycub_head_move", fields: { ACT: "look_down", DEG: 15, SPD: "1.5" } },
            { kind: "block", type: "pycub_head_move", fields: { ACT: "turn_head", DEG: 20, SPD: "1.5" } },
            { kind: "block", type: "pycub_body_move", fields: { ACT: "bend_forward", DEG: 15, SPD: "1.5" } },
            { kind: "block", type: "pycub_body_move", fields: { ACT: "turn_body", DEG: 15, SPD: "1.5" } }
          ]
        },
        {
          kind: "category",
          name: L("cat_sensors"),
          colour: "#d97706",
          contents: [
            { kind: "block", type: "pycub_ball_touched" },
            { kind: "block", type: "pycub_skin_touched" },
            { kind: "block", type: "pycub_ball_distance" },
            { kind: "block", type: "pycub_ball_hits" }
          ]
        },
        {
          kind: "category",
          name: L("cat_loops"),
          colour: "#16a34a",
          contents: [
            {
              kind: "block",
              type: "controls_repeat_ext",
              inputs: { TIMES: { shadow: { type: "math_number", fields: { NUM: 3 } } } }
            },
            { kind: "block", type: "controls_whileUntil" },
            { kind: "block", type: "pycub_forever" }
          ]
        },
        {
          kind: "category",
          name: L("cat_logic"),
          colour: "#0284c7",
          contents: [
            { kind: "block", type: "controls_if" },
            { kind: "block", type: "logic_compare" },
            { kind: "block", type: "logic_operation" },
            { kind: "block", type: "logic_negate" },
            { kind: "block", type: "logic_boolean" }
          ]
        },
        {
          kind: "category",
          name: L("cat_math"),
          colour: "#4f46e5",
          contents: [
            { kind: "block", type: "math_number", fields: { NUM: 15 } },
            { kind: "block", type: "math_arithmetic" },
            { kind: "block", type: "math_random_int" },
            {
              kind: "block",
              type: "text_print",
              inputs: { TEXT: { shadow: { type: "text", fields: { TEXT: "Hello iCub!" } } } }
            }
          ]
        },
        {
          kind: "category",
          name: L("cat_vars"),
          colour: "#db2777",
          custom: "VARIABLE"
        }
      ]
    };
  }

  window.PyCubBlocks = {
    setLang: function (lang) {
      CURRENT_LANG = lang === "cs" ? "cs" : "en";
      registerBlocks();
      registerGenerators();
    },
    getToolbox: buildToolbox,
    generateClean: function (workspace) {
      var py = (window.python && window.python.pythonGenerator) ? window.python.pythonGenerator : Blockly.Python;
      py.STATEMENT_PREFIX = null;
      var body = py.workspaceToCode(workspace) || "";
      return "from icub_pybullet.kid_api import *\n\n" + body;
    },
    generateInstrumented: function (workspace) {
      var py = (window.python && window.python.pythonGenerator) ? window.python.pythonGenerator : Blockly.Python;
      py.STATEMENT_PREFIX = "_highlight_block(%1)\n";
      var body = py.workspaceToCode(workspace) || "";
      py.STATEMENT_PREFIX = null;
      return body;
    }
  };
})();
