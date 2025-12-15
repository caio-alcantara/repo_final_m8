import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { BarChart, Bar, XAxis, YAxis, CartesianGrid, ResponsiveContainer, Tooltip, Cell } from "recharts";

// Dados mockados - preparados para futura integração com API
const stateData = [
  { estado: "SP", visitantes: 58 },
  { estado: "RJ", visitantes: 15 },
  { estado: "MG", visitantes: 10 },
  { estado: "PR", visitantes: 8 },
  { estado: "Outros", visitantes: 9 },
];

const cityData = [
  { cidade: "São Paulo", visitantes: 35 },
  { cidade: "Guarulhos", visitantes: 12 },
  { cidade: "Santo André", visitantes: 8 },
  { cidade: "Osasco", visitantes: 6 },
  { cidade: "Outras", visitantes: 39 },
];

const interestData = [
  { curso: "Eng. Comp.", visitantes: 42 },
  { curso: "Eng. Soft.", visitantes: 28 },
  { curso: "Ciência Dados", visitantes: 18 },
  { curso: "Outros", visitantes: 12 },
];

const StateChart = () => (
  <Card className="h-full flex flex-col">
    <CardHeader className="pb-1 pt-2 px-3 shrink-0">
      <CardTitle className="text-xs font-semibold text-foreground">
        Distribuição por estado
      </CardTitle>
      <p className="text-[10px] text-muted-foreground">Nº de visitantes por UF</p>
    </CardHeader>
    <CardContent className="pt-0 px-3 pb-2 flex-1 min-h-0">
      <ResponsiveContainer width="100%" height="100%">
        <BarChart data={stateData} layout="vertical" margin={{ top: 0, right: 15, left: -5, bottom: 0 }}>
          <CartesianGrid strokeDasharray="3 3" stroke="hsl(var(--border))" horizontal={false} />
          <XAxis 
            type="number"
            tick={{ fontSize: 8, fill: 'hsl(var(--muted-foreground))' }}
            axisLine={{ stroke: 'hsl(var(--border))' }}
            tickLine={{ stroke: 'hsl(var(--border))' }}
          />
          <YAxis 
            type="category"
            dataKey="estado"
            tick={{ fontSize: 8, fill: 'hsl(var(--muted-foreground))' }}
            axisLine={{ stroke: 'hsl(var(--border))' }}
            tickLine={{ stroke: 'hsl(var(--border))' }}
            width={35}
          />
          <Tooltip 
            contentStyle={{ 
              backgroundColor: 'hsl(var(--card))', 
              border: '1px solid hsl(var(--border))',
              borderRadius: '6px',
              fontSize: '10px'
            }}
          />
          <Bar dataKey="visitantes" radius={[0, 4, 4, 0]}>
            {stateData.map((_, index) => (
              <Cell key={`cell-${index}`} fill="hsl(var(--chart-purple))" />
            ))}
          </Bar>
        </BarChart>
      </ResponsiveContainer>
    </CardContent>
  </Card>
);

const CityChart = () => (
  <Card className="h-full flex flex-col">
    <CardHeader className="pb-1 pt-2 px-3 shrink-0">
      <CardTitle className="text-xs font-semibold text-foreground">
        Cidades com mais visitantes
      </CardTitle>
      <p className="text-[10px] text-muted-foreground">Top 5 cidades</p>
    </CardHeader>
    <CardContent className="pt-0 px-3 pb-2 flex-1 min-h-0">
      <ResponsiveContainer width="100%" height="100%">
        <BarChart data={cityData} layout="vertical" margin={{ top: 0, right: 15, left: -5, bottom: 0 }}>
          <CartesianGrid strokeDasharray="3 3" stroke="hsl(var(--border))" horizontal={false} />
          <XAxis 
            type="number"
            tick={{ fontSize: 8, fill: 'hsl(var(--muted-foreground))' }}
            axisLine={{ stroke: 'hsl(var(--border))' }}
            tickLine={{ stroke: 'hsl(var(--border))' }}
          />
          <YAxis 
            type="category"
            dataKey="cidade"
            tick={{ fontSize: 8, fill: 'hsl(var(--muted-foreground))' }}
            axisLine={{ stroke: 'hsl(var(--border))' }}
            tickLine={{ stroke: 'hsl(var(--border))' }}
            width={55}
          />
          <Tooltip 
            contentStyle={{ 
              backgroundColor: 'hsl(var(--card))', 
              border: '1px solid hsl(var(--border))',
              borderRadius: '6px',
              fontSize: '10px'
            }}
          />
          <Bar dataKey="visitantes" radius={[0, 4, 4, 0]}>
            {cityData.map((_, index) => (
              <Cell key={`cell-${index}`} fill="hsl(var(--chart-purple))" />
            ))}
          </Bar>
        </BarChart>
      </ResponsiveContainer>
    </CardContent>
  </Card>
);

const InterestChart = () => (
  <Card className="h-full flex flex-col">
    <CardHeader className="pb-1 pt-2 px-3 shrink-0">
      <CardTitle className="text-xs font-semibold text-foreground">
        Interesse declarado
      </CardTitle>
      <p className="text-[10px] text-muted-foreground">Cursos/áreas de interesse</p>
    </CardHeader>
    <CardContent className="pt-0 px-3 pb-2 flex-1 min-h-0">
      <ResponsiveContainer width="100%" height="100%">
        <BarChart data={interestData} margin={{ top: 0, right: 5, left: -20, bottom: 0 }}>
          <CartesianGrid strokeDasharray="3 3" stroke="hsl(var(--border))" vertical={false} />
          <XAxis 
            dataKey="curso"
            tick={{ fontSize: 7, fill: 'hsl(var(--muted-foreground))' }}
            axisLine={{ stroke: 'hsl(var(--border))' }}
            tickLine={{ stroke: 'hsl(var(--border))' }}
            interval={0}
          />
          <YAxis 
            tick={{ fontSize: 8, fill: 'hsl(var(--muted-foreground))' }}
            axisLine={{ stroke: 'hsl(var(--border))' }}
            tickLine={{ stroke: 'hsl(var(--border))' }}
          />
          <Tooltip 
            contentStyle={{ 
              backgroundColor: 'hsl(var(--card))', 
              border: '1px solid hsl(var(--border))',
              borderRadius: '6px',
              fontSize: '10px'
            }}
          />
          <Bar dataKey="visitantes" radius={[4, 4, 0, 0]}>
            {interestData.map((_, index) => (
              <Cell key={`cell-${index}`} fill="hsl(var(--chart-purple))" />
            ))}
          </Bar>
        </BarChart>
      </ResponsiveContainer>
    </CardContent>
  </Card>
);

const VisitorsProfileChart = () => {
  return (
    <div className="grid grid-rows-3 gap-3 h-full">
      <StateChart />
      <CityChart />
      <InterestChart />
    </div>
  );
};

export default VisitorsProfileChart;
