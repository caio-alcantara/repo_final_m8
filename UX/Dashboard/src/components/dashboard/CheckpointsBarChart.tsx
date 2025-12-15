import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { BarChart, Bar, XAxis, YAxis, CartesianGrid, ResponsiveContainer, Tooltip, Cell } from "recharts";

const data = [
  { checkpoint: "Checkpoint 1", perguntas: 100 },
  { checkpoint: "Checkpoint 2", perguntas: 80 },
  { checkpoint: "Checkpoint 3", perguntas: 25 },
];

const CheckpointsBarChart = () => {
  return (
    <Card className="h-full">
      <CardHeader className="pb-1 pt-3 px-4">
        <CardTitle className="text-sm font-semibold text-foreground">
          Checkpoints com maior concentração de perguntas
        </CardTitle>
      </CardHeader>
      <CardContent className="pt-0 px-4 pb-3">
        <ResponsiveContainer width="100%" height={150}>
          <BarChart data={data} layout="vertical" margin={{ top: 5, right: 10, left: 10, bottom: 0 }}>
            <CartesianGrid strokeDasharray="3 3" stroke="hsl(var(--border))" horizontal={false} />
            <XAxis 
              type="number"
              tick={{ fontSize: 11, fill: 'hsl(var(--muted-foreground))' }}
              axisLine={{ stroke: 'hsl(var(--border))' }}
              tickLine={{ stroke: 'hsl(var(--border))' }}
              domain={[0, 100]}
              ticks={[0, 25, 50, 75, 100]}
            />
            <YAxis 
              type="category"
              dataKey="checkpoint"
              tick={{ fontSize: 11, fill: 'hsl(var(--muted-foreground))' }}
              axisLine={{ stroke: 'hsl(var(--border))' }}
              tickLine={{ stroke: 'hsl(var(--border))' }}
              width={80}
            />
            <Tooltip 
              contentStyle={{ 
                backgroundColor: 'hsl(var(--card))', 
                border: '1px solid hsl(var(--border))',
                borderRadius: '6px',
                fontSize: '12px'
              }}
            />
            <Bar dataKey="perguntas" radius={[0, 4, 4, 0]}>
              {data.map((_, index) => (
                <Cell key={`cell-${index}`} fill="hsl(var(--chart-purple))" />
              ))}
            </Bar>
          </BarChart>
        </ResponsiveContainer>
        <div className="text-center text-xs text-muted-foreground mt-1">Nº de perguntas</div>
      </CardContent>
    </Card>
  );
};

export default CheckpointsBarChart;
