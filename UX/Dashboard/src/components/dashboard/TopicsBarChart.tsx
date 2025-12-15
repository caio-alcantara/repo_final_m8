import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { BarChart, Bar, XAxis, YAxis, CartesianGrid, ResponsiveContainer, Tooltip, Cell } from "recharts";

const data = [
  { categoria: "Metodologia", perguntas: 60 },
  { categoria: "Processo seletivo", perguntas: 95 },
  { categoria: "Clubes", perguntas: 90 },
  { categoria: "Campus", perguntas: 40 },
  { categoria: "Dia-a-dia do aluno", perguntas: 30 },
];

const TopicsBarChart = () => {
  return (
    <Card className="h-full">
      <CardHeader className="pb-1 pt-3 px-4">
        <CardTitle className="text-sm font-semibold text-foreground">
          Temas mais perguntados
        </CardTitle>
      </CardHeader>
      <CardContent className="pt-0 px-4 pb-3">
        <ResponsiveContainer width="100%" height={150}>
          <BarChart data={data} margin={{ top: 5, right: 10, left: -20, bottom: 0 }}>
            <CartesianGrid strokeDasharray="3 3" stroke="hsl(var(--border))" vertical={false} />
            <XAxis 
              dataKey="categoria" 
              tick={{ fontSize: 9, fill: 'hsl(var(--muted-foreground))' }}
              axisLine={{ stroke: 'hsl(var(--border))' }}
              tickLine={{ stroke: 'hsl(var(--border))' }}
              interval={0}
            />
            <YAxis 
              tick={{ fontSize: 11, fill: 'hsl(var(--muted-foreground))' }}
              axisLine={{ stroke: 'hsl(var(--border))' }}
              tickLine={{ stroke: 'hsl(var(--border))' }}
              domain={[0, 100]}
              ticks={[0, 25, 50, 75, 100]}
              label={{ value: 'Nº de perguntas', angle: -90, position: 'insideLeft', fontSize: 10, fill: 'hsl(var(--muted-foreground))', dx: 10 }}
            />
            <Tooltip 
              contentStyle={{ 
                backgroundColor: 'hsl(var(--card))', 
                border: '1px solid hsl(var(--border))',
                borderRadius: '6px',
                fontSize: '12px'
              }}
            />
            <Bar dataKey="perguntas" radius={[4, 4, 0, 0]}>
              {data.map((_, index) => (
                <Cell key={`cell-${index}`} fill="hsl(var(--chart-purple))" />
              ))}
            </Bar>
          </BarChart>
        </ResponsiveContainer>
        <div className="text-center text-xs text-muted-foreground mt-1">Categoria</div>
      </CardContent>
    </Card>
  );
};

export default TopicsBarChart;
