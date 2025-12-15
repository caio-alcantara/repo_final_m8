import { useState } from "react";
import AppHeader from "@/components/AppHeader";
import UserHeader from "@/components/user-management/UserHeader";
import { Input } from "@/components/ui/input";
import { Button } from "@/components/ui/button";
import { Table, TableBody, TableCell, TableHead, TableHeader, TableRow } from "@/components/ui/table";
import { Search, Plus, Pencil, Trash2 } from "lucide-react";

// Mock data
const INITIAL_USERS = [
  { id: 1, email: "ana@inteli.edu.br", status: "Ativo" },
  { id: 2, email: "luis@inteli.edu.br", status: "Inativo" },
  { id: 3, email: "flavia@inteli.edu.br", status: "Ativo" },
  { id: 4, email: "pedro@inteli.edu.br", status: "Ativo" },
  { id: 5, email: "julia@inteli.edu.br", status: "Inativo" },
];

const UsersManagement = () => {
  const [searchTerm, setSearchTerm] = useState("");
  const [users, setUsers] = useState(INITIAL_USERS);

  const filteredUsers = users.filter(user => 
    user.email.toLowerCase().includes(searchTerm.toLowerCase())
  );

  const handleDelete = (id: number) => {
    // Integração
    setUsers(users.filter(user => user.id !== id));
  };

  return (
    <div className="h-screen bg-background flex flex-col overflow-hidden">
      <AppHeader/>
      <UserHeader />
      
      <main className="flex-1 max-w-7xl w-full mx-auto px-8 py-8 overflow-auto">
        <div className="relative flex flex-col gap-6">
          
          <div className="relative w-full">
            <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 text-muted-foreground w-4 h-4" />
            <Input 
              placeholder="Buscar" 
              className="pl-10 bg-white border-gray-200 rounded-full h-12 text-base shadow-sm"
              value={searchTerm}
              onChange={(e) => setSearchTerm(e.target.value)}
            />
          </div>

          <div className="absolute right-0 top-[60px] z-10">
             <Button 
                size="icon" 
                className="h-12 w-12 rounded-full bg-primary hover:bg-primary/90 shadow-lg"
             >
                <Plus className="h-6 w-6 text-white" />
             </Button>
          </div>

          {/* Table */}
          <div className="bg-white rounded-lg border border-gray-100 shadow-sm mt-4">
            <Table>
              <TableHeader>
                <TableRow className="hover:bg-transparent border-b border-gray-100">
                  <TableHead className="w-[50%] py-6 text-gray-900 font-medium pl-6">Usuário</TableHead>
                  <TableHead className="w-[30%] py-6 text-gray-900 font-medium">Status</TableHead>
                  <TableHead className="w-[20%] py-6 text-gray-900 font-medium text-right pr-6">Ações</TableHead>
                </TableRow>
              </TableHeader>
              <TableBody>
                {filteredUsers.map((user) => (
                  <TableRow key={user.id} className="hover:bg-gray-50 border-b border-gray-100 last:border-0">
                    <TableCell className="py-6 pl-6 text-gray-600">{user.email}</TableCell>
                    <TableCell className="py-6 text-gray-600">{user.status}</TableCell>
                    <TableCell className="py-6 pr-6 text-right">
                      <div className="flex justify-end gap-3">
                        <button className="text-gray-400 hover:text-gray-900 transition-colors">
                          <Pencil className="w-5 h-5" />
                        </button>
                        <button 
                          onClick={() => handleDelete(user.id)}
                          className="text-gray-400 hover:text-red-500 transition-colors"
                        >
                          <Trash2 className="w-5 h-5" />
                        </button>
                      </div>
                    </TableCell>
                  </TableRow>
                ))}
              </TableBody>
            </Table>

            {filteredUsers.length === 0 && (
              <div className="p-8 text-center text-gray-500">
                Nenhum usuário encontrado.
              </div>
            )}
          </div>

        </div>
      </main>
    </div>
  );
};

export default UsersManagement;