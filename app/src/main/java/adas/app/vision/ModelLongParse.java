package adas.app.vision;

/** Parse lead + PLAN velocity from supercombo flat output (6409).
 *
 *  Layout after ROAD_END=5755 matches flowpilot F2 lead block:
 *    LEAD (105) + short meta gap + POSE (12) + TEMPORAL (512).
 */
public final class ModelLongParse {
    public static final int ROAD_END = 5755;
    public static final int LEAD_IDX = ROAD_END;
    public static final int LEAD_MHP_N = 2;
    public static final int LEAD_TRAJ_LEN = 6;
    public static final int LEAD_PRED_DIM = 4;
    public static final int LEAD_MHP_VALS = LEAD_PRED_DIM * LEAD_TRAJ_LEN; // 24
    public static final int LEAD_MHP_SELECTION = 3;
    public static final int LEAD_MHP_GROUP = 2 * LEAD_MHP_VALS + LEAD_MHP_SELECTION; // 51
    public static final int LEAD_PROB_IDX = LEAD_IDX + LEAD_MHP_N * LEAD_MHP_GROUP; // 5857

    public static final int PLAN_MHP_N = 5;
    public static final int PLAN_COLS = 15;
    public static final int PLAN_GROUP = 2 * PLAN_COLS * 33 + 1; // 991

    public static final class Lead {
        public float prob;
        public float probTime;
        public final float[] x = new float[LEAD_TRAJ_LEN];
        public final float[] y = new float[LEAD_TRAJ_LEN];
        public final float[] v = new float[LEAD_TRAJ_LEN];
        public final float[] a = new float[LEAD_TRAJ_LEN];
    }

    public static final class Out {
        public final float[] planVx = new float[33];
        public final float[] planVy = new float[33];
        public final float[] planVz = new float[33];
        public final Lead lead0 = new Lead();
        public final Lead lead1 = new Lead();
        public final Lead lead2 = new Lead();
        public boolean ok;
    }

    private ModelLongParse() {}

    private static float sigmoid(float x) {
        if (x >= 0) {
            float z = (float) Math.exp(-x);
            return 1.f / (1.f + z);
        }
        float z = (float) Math.exp(x);
        return z / (1.f + z);
    }

    private static void fillLead(Lead lead, float[] out, int tOffset, float probTime) {
        lead.probTime = probTime;
        lead.prob = sigmoid(out[LEAD_PROB_IDX + tOffset]);
        // Pick MHP hyp by selection logit at group_end + (tOffset - LEAD_MHP_SELECTION)
        int selOff = tOffset - LEAD_MHP_SELECTION;
        int best = 0;
        float bestLogit = Float.NEGATIVE_INFINITY;
        for (int i = 0; i < LEAD_MHP_N; i++) {
            // Selection logits sit at the end of each MHP group (flowpilot get_lead_data).
            float logit = out[LEAD_IDX + (i + 1) * LEAD_MHP_GROUP + selOff];
            if (logit > bestLogit) {
                bestLogit = logit;
                best = i;
            }
        }
        int base = LEAD_IDX + best * LEAD_MHP_GROUP;
        for (int i = 0; i < LEAD_TRAJ_LEN; i++) {
            int row = base + i * LEAD_PRED_DIM;
            lead.x[i] = out[row];
            lead.y[i] = out[row + 1];
            lead.v[i] = out[row + 2];
            lead.a[i] = out[row + 3];
        }
    }

    public static Out parse(float[] out) {
        Out o = new Out();
        if (out == null || out.length < LEAD_PROB_IDX + 2) {
            return o;
        }
        // Best PLAN velocity
        int bestHyp = 0;
        float bestLogit = Float.NEGATIVE_INFINITY;
        for (int i = 0; i < PLAN_MHP_N; i++) {
            float logit = out[(i + 1) * PLAN_GROUP - 1];
            if (logit > bestLogit) {
                bestLogit = logit;
                bestHyp = i;
            }
        }
        int planBase = bestHyp * PLAN_GROUP;
        for (int i = 0; i < 33; i++) {
            int row = planBase + i * PLAN_COLS;
            o.planVx[i] = out[row + 3];
            o.planVy[i] = out[row + 4];
            o.planVz[i] = out[row + 5];
        }
        fillLead(o.lead0, out, 0, 0.f);
        fillLead(o.lead1, out, 1, 2.f);
        fillLead(o.lead2, out, 2, 4.f);
        o.ok = true;
        return o;
    }
}
