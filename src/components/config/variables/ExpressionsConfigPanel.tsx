import { Divider } from "@mui/material";
import { observer } from "mobx-react";
import { Component } from "react";
import { doc } from "../../../document/DocumentManager";
import PoseVariablesConfigPanel from "./PoseVariablesConfigPanel";
import VariablesConfigPanel, {
  GeneralVariableAddPanel
} from "./VariablesConfigPanel";

type Props = object;

type State = object;

class ExpressionsConfigPanel extends Component<Props, State> {
  rowGap = 4;

  render() {
    doc.variables.expressions.keys();
    return (
      <div
        style={{
          rowGap: `${1 * this.rowGap}px`,
          columnGap: this.rowGap,
          fontSize: "1rem",
          margin: `${1 * this.rowGap}px`,
          display: "grid",
          gridTemplateColumns:
            "max-content max-content max-content max-content",
          width: "max-content"
        }}
      >
        <VariablesConfigPanel></VariablesConfigPanel>
        {doc.variables.poses.size > 0 && (
          <Divider sx={{ color: "gray", gridColumn: "1 / -1" }}>
            POSE VARIABLES
          </Divider>
        )}
        <PoseVariablesConfigPanel></PoseVariablesConfigPanel>
        <Divider sx={{ color: "gray", gridColumn: "1 / -1" }}>
          ADD NEW VARIABLE
        </Divider>
        <GeneralVariableAddPanel></GeneralVariableAddPanel>
      </div>
    );
  }
}
export default observer(ExpressionsConfigPanel);
